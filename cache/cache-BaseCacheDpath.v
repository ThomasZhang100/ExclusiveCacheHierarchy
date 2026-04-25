//=========================================================================
// Exclusive Cache Datapath
//=========================================================================
`ifndef CACHE_BASE_CACHE_DPATH_V
`define CACHE_BASE_CACHE_DPATH_V

module cache_BaseCacheDpath
#(
  parameter p_num_sets  = 256,
  parameter p_num_ways  = 1,
  parameter p_line_sz   = 16,
  parameter p_addr_sz   = 32,
  parameter p_data_sz   = 32
)(
  input clk,
  input reset,

  input  [p_addr_sz-1:0]  refill_addr,
  input  [p_data_sz-1:0]  up_req_wdata,
  input                   up_req_type,
  input  [1:0]            up_req_len,    // 0=word, 1=byte, 2=halfword
  output [p_data_sz-1:0]  up_resp_rdata,

  input  [p_addr_sz-1:0]  incoming_victim_addr,

  input  [p_num_ways-1:0] refill_tag_wen,
  input  [p_num_ways-1:0] refill_data_wen,
  input  [p_line_sz*8-1:0] refill_line,
  input  [p_num_ways-1:0] refill_invalidate_way,
  input                   mark_dirty,
  input  [p_num_ways-1:0] store_hit_data_wen,   // store hit: update word in hit way

  input  [p_num_ways-1:0] victim_tag_wen,
  input  [p_num_ways-1:0] victim_data_wen,
  input  [p_line_sz*8-1:0] incoming_victim_data,
  input                   incoming_victim_dirty, // dirty bit from upstream SWAP message
  input  [p_num_ways-1:0] victim_set_evict_way,

  input [$clog2(p_num_ways > 1 ? p_num_ways : 2)-1:0] lru_update_refill_way,
  input                                                 lru_update_refill_en,
  input [$clog2(p_num_ways > 1 ? p_num_ways : 2)-1:0] lru_update_victim_way,
  input                                                 lru_update_victim_en,

  output                                                hit,
  output [$clog2(p_num_ways > 1 ? p_num_ways : 2)-1:0] hit_way,

  output [$clog2(p_num_ways > 1 ? p_num_ways : 2)-1:0] refill_lru_way,
  output [p_line_sz*8-1:0] hit_line,           // data_rd_refill[hit_way]
  output [p_addr_sz-1:0]   refill_evict_addr,
  output [p_line_sz*8-1:0] refill_evict_line,
  output                    refill_evict_dirty,
  output                    refill_evict_valid,

  output [$clog2(p_num_sets > 1 ? p_num_sets : 2)-1:0] victim_set_idx,
  output                    same_set,
  output                    victim_set_has_free,
  output [$clog2(p_num_ways > 1 ? p_num_ways : 2)-1:0] victim_free_way,
  output [$clog2(p_num_ways > 1 ? p_num_ways : 2)-1:0] victim_lru_way,
  output [p_addr_sz-1:0]   victim_set_evict_addr,
  output [p_line_sz*8-1:0] victim_set_evict_line,
  output                    victim_set_evict_dirty
);

  localparam c_offset_sz    = $clog2(p_line_sz);
  localparam c_idx_sz       = $clog2(p_num_sets > 1 ? p_num_sets : 2);
  localparam c_tag_sz       = p_addr_sz - c_idx_sz - c_offset_sz;
  localparam c_line_bits    = p_line_sz * 8;
  localparam c_way_bits     = $clog2(p_num_ways > 1 ? p_num_ways : 2);
  localparam c_set_bits     = $clog2(p_num_sets > 1 ? p_num_sets : 2);
  localparam c_words_per_line = p_line_sz / (p_data_sz/8);

  // Address decomposition
  wire [c_idx_sz-1:0]  req_set_idx = refill_addr[c_offset_sz+c_idx_sz-1 : c_offset_sz];
  wire [c_tag_sz-1:0]  req_tag     = refill_addr[p_addr_sz-1 : c_offset_sz+c_idx_sz];
  wire [c_offset_sz-1:0] req_offset = refill_addr[c_offset_sz-1:0];

  wire [c_idx_sz-1:0]  vic_set_idx = incoming_victim_addr[c_offset_sz+c_idx_sz-1 : c_offset_sz];
  wire [c_tag_sz-1:0]  vic_tag     = incoming_victim_addr[p_addr_sz-1 : c_offset_sz+c_idx_sz];

  assign victim_set_idx = vic_set_idx;
  assign same_set       = (vic_set_idx == req_set_idx);

  //----------------------------------------------------------------------
  // Tag SRAM: {dirty(1), valid(1), tag(c_tag_sz)}
  //----------------------------------------------------------------------

  localparam c_tag_entry_sz = 1 + 1 + c_tag_sz; // dirty | valid | tag

  reg [c_tag_entry_sz-1:0] tag_array [0:p_num_sets-1][0:p_num_ways-1];

  // Combinational reads for both sets
  wire [c_tag_entry_sz-1:0] tag_rd_refill [0:p_num_ways-1];
  wire [c_tag_entry_sz-1:0] tag_rd_victim [0:p_num_ways-1];

  genvar gw;
  generate
    for (gw = 0; gw < p_num_ways; gw = gw + 1) begin : g_tag_rd
      assign tag_rd_refill[gw] = tag_array[req_set_idx][gw];
      assign tag_rd_victim[gw] = tag_array[vic_set_idx][gw];
    end
  endgenerate

  // Synchronous tag writes
  integer i;
  always @(posedge clk) begin
    for (i = 0; i < p_num_ways; i = i + 1) begin
      // Refill-set: write new tag (refill or inplace-swap)
      if (refill_tag_wen[i])
        tag_array[req_set_idx][i] <= {mark_dirty, 1'b1, req_tag};
      // Refill-set: invalidate (line leaving upward to L1)
      if (refill_invalidate_way[i])
        tag_array[req_set_idx][i][c_tag_entry_sz-2] <= 1'b0; // clear valid
      // Victim-set: place incoming victim, preserving its dirty bit
      if (victim_tag_wen[i])
        tag_array[vic_set_idx][i] <= {incoming_victim_dirty, 1'b1, vic_tag};
      // Victim-set: evict V3 before placing V
      if (victim_set_evict_way[i])
        tag_array[vic_set_idx][i][c_tag_entry_sz-2] <= 1'b0; // clear valid
      // Store hit: mark the hit line dirty without changing tag or valid
      if (store_hit_data_wen[i])
        tag_array[req_set_idx][i][c_tag_entry_sz-1] <= 1'b1;
    end
  end

  //----------------------------------------------------------------------
  // Data SRAM
  //----------------------------------------------------------------------

  reg [c_line_bits-1:0] data_array [0:p_num_sets-1][0:p_num_ways-1];

  wire [c_line_bits-1:0] data_rd_refill [0:p_num_ways-1];
  wire [c_line_bits-1:0] data_rd_victim [0:p_num_ways-1];

  generate
    for (gw = 0; gw < p_num_ways; gw = gw + 1) begin : g_data_rd
      assign data_rd_refill[gw] = data_array[req_set_idx][gw];
      assign data_rd_victim[gw] = data_array[vic_set_idx][gw];
    end
  endgenerate

  // Synchronous data writes
  integer j;
  always @(posedge clk) begin
    for (j = 0; j < p_num_ways; j = j + 1) begin
      if (refill_data_wen[j]) begin
        // Write-allocate: merge store bytes into refill line when marking dirty
        if (mark_dirty) begin
          data_array[req_set_idx][j] <= refill_line;
          if (up_req_len == 2'd1)
            data_array[req_set_idx][j][req_offset[c_offset_sz-1:2]*p_data_sz + req_offset[1:0]*8 +: 8]  <= up_req_wdata[7:0];
          else if (up_req_len == 2'd2)
            data_array[req_set_idx][j][req_offset[c_offset_sz-1:2]*p_data_sz + req_offset[1:0]*8 +: 16] <= up_req_wdata[15:0];
          else
            data_array[req_set_idx][j][req_offset[c_offset_sz-1:2]*p_data_sz +: p_data_sz] <= up_req_wdata;
        end else begin
          data_array[req_set_idx][j] <= refill_line;
        end
      end
      if (victim_data_wen[j])
        data_array[vic_set_idx][j] <= incoming_victim_data;
      // Store hit: overwrite only the target byte(s) within the hit line
      if (store_hit_data_wen[j]) begin
        if (up_req_len == 2'd1)
          data_array[req_set_idx][j][req_offset[c_offset_sz-1:2]*p_data_sz + req_offset[1:0]*8 +: 8]  <= up_req_wdata[7:0];
        else if (up_req_len == 2'd2)
          data_array[req_set_idx][j][req_offset[c_offset_sz-1:2]*p_data_sz + req_offset[1:0]*8 +: 16] <= up_req_wdata[15:0];
        else
          data_array[req_set_idx][j][req_offset[c_offset_sz-1:2]*p_data_sz +: p_data_sz] <= up_req_wdata;
      end
    end
  end

  //----------------------------------------------------------------------
  // Hit detection (combinational)
  //----------------------------------------------------------------------

  reg                  hit_r;
  reg [c_way_bits-1:0] hit_way_r;

  always @(*) begin
    hit_r     = 1'b0;
    hit_way_r = {c_way_bits{1'b0}};
    for (i = 0; i < p_num_ways; i = i + 1) begin
      if (tag_rd_refill[i][c_tag_entry_sz-2] &&        // valid
          tag_rd_refill[i][c_tag_sz-1:0] == req_tag) begin // tag match
        hit_r     = 1'b1;
        hit_way_r = i[c_way_bits-1:0];
      end
    end
  end

  assign hit     = hit_r;
  assign hit_way = hit_way_r;

  //----------------------------------------------------------------------
  // Tree Pseudo-LRU replacement
  //----------------------------------------------------------------------
  // For p_num_ways ways (must be a power of 2), each set stores
  // p_num_ways-1 bits as a complete binary tree of internal nodes:
  //
  //   bit index 0          → root
  //   bit index 2i+1       → left  child of node i
  //   bit index 2i+2       → right child of node i
  //
  // Bit convention: 0 = left subtree is the eviction candidate (LRU lives
  //                     there); 1 = right subtree is the eviction candidate.
  //
  // On ACCESS of way W (log2(ways) levels):
  //   At each tree level, check which subtree W belongs to and flip the
  //   bit at that node to point AWAY from W, making the other subtree the
  //   new LRU candidate.
  //
  // On EVICTION:
  //   Follow bits root→leaf; 0 → go left, 1 → go right.
  //   The accumulated path bits form the LRU way number (MSB first).
  //

  localparam c_plru_levels = $clog2(p_num_ways > 1 ? p_num_ways : 2);
  // At least 1 bit to keep Verilog arrays legal for the direct-mapped case.
  localparam c_plru_bits   = (p_num_ways > 1) ? (p_num_ways - 1) : 1;

  // One tree per set, shared for both refill-set and victim-set lookups.
  // Reads are indexed by req_set_idx or vic_set_idx; updates by whichever
  // set the FSM is currently touching (they are in distinct states, so
  // there is never a write-write conflict).
  reg [c_plru_bits-1:0] plru [0:p_num_sets-1];

  // ---- Eviction: combinational traversal → LRU way ----------------------

  reg [c_way_bits-1:0] refill_lru_way_r;
  reg [c_way_bits-1:0] victim_lru_way_r;

  integer plru_r_node, plru_r_lvl;
  integer plru_v_node, plru_v_lvl;

  always @(*) begin
    plru_r_node      = 0;
    refill_lru_way_r = {c_way_bits{1'b0}};
    for (plru_r_lvl = 0; plru_r_lvl < c_plru_levels; plru_r_lvl = plru_r_lvl + 1) begin
      if (plru[req_set_idx][plru_r_node] == 1'b0) begin
        refill_lru_way_r[c_plru_levels - 1 - plru_r_lvl] = 1'b0;
        plru_r_node = 2 * plru_r_node + 1;
      end else begin
        refill_lru_way_r[c_plru_levels - 1 - plru_r_lvl] = 1'b1;
        plru_r_node = 2 * plru_r_node + 2;
      end
    end
  end

  always @(*) begin
    plru_v_node      = 0;
    victim_lru_way_r = {c_way_bits{1'b0}};
    for (plru_v_lvl = 0; plru_v_lvl < c_plru_levels; plru_v_lvl = plru_v_lvl + 1) begin
      if (plru[vic_set_idx][plru_v_node] == 1'b0) begin
        victim_lru_way_r[c_plru_levels - 1 - plru_v_lvl] = 1'b0;
        plru_v_node = 2 * plru_v_node + 1;
      end else begin
        victim_lru_way_r[c_plru_levels - 1 - plru_v_lvl] = 1'b1;
        plru_v_node = 2 * plru_v_node + 2;
      end
    end
  end

  assign refill_lru_way = refill_lru_way_r;
  assign victim_lru_way = victim_lru_way_r;

  // ---- Update: flip bits along the path to the accessed way -------------

  integer k;
  integer plru_upd_node, plru_upd_lvl, plru_upd_bit;

  initial begin
    for (k = 0; k < p_num_sets; k = k + 1)
      plru[k] = {c_plru_bits{1'b0}};
  end

  always @(posedge clk) begin
    if (reset) begin
      for (k = 0; k < p_num_sets; k = k + 1)
        plru[k] <= {c_plru_bits{1'b0}};
    end else begin
      if (lru_update_refill_en) begin
        plru_upd_node = 0;
        for (plru_upd_lvl = 0; plru_upd_lvl < c_plru_levels; plru_upd_lvl = plru_upd_lvl + 1) begin
          plru_upd_bit = (lru_update_refill_way >> (c_plru_levels - 1 - plru_upd_lvl)) & 1;
          if (plru_upd_bit == 0) begin
            plru[req_set_idx][plru_upd_node] <= 1'b1; // W in left  → right is now LRU
            plru_upd_node = 2 * plru_upd_node + 1;
          end else begin
            plru[req_set_idx][plru_upd_node] <= 1'b0; // W in right → left  is now LRU
            plru_upd_node = 2 * plru_upd_node + 2;
          end
        end
      end
      if (lru_update_victim_en) begin
        plru_upd_node = 0;
        for (plru_upd_lvl = 0; plru_upd_lvl < c_plru_levels; plru_upd_lvl = plru_upd_lvl + 1) begin
          plru_upd_bit = (lru_update_victim_way >> (c_plru_levels - 1 - plru_upd_lvl)) & 1;
          if (plru_upd_bit == 0) begin
            plru[vic_set_idx][plru_upd_node] <= 1'b1;
            plru_upd_node = 2 * plru_upd_node + 1;
          end else begin
            plru[vic_set_idx][plru_upd_node] <= 1'b0;
            plru_upd_node = 2 * plru_upd_node + 2;
          end
        end
      end
    end
  end

  //----------------------------------------------------------------------
  // Refill-set eviction info
  //----------------------------------------------------------------------

  wire [c_tag_entry_sz-1:0] refill_evict_entry = tag_rd_refill[refill_lru_way];
  wire [c_line_bits-1:0]    refill_evict_data_w = data_rd_refill[refill_lru_way];

  assign refill_evict_valid = refill_evict_entry[c_tag_entry_sz-2]; // valid bit
  assign refill_evict_dirty = refill_evict_entry[c_tag_entry_sz-1]; // dirty bit
  assign refill_evict_line  = refill_evict_data_w;
  assign refill_evict_addr  = {refill_evict_entry[c_tag_sz-1:0], req_set_idx, {c_offset_sz{1'b0}}};

  //----------------------------------------------------------------------
  // Victim-set free-way scan
  //----------------------------------------------------------------------

  reg                   vsfree_r;
  reg [c_way_bits-1:0]  vsfree_way_r;

  always @(*) begin
    vsfree_r     = 1'b0;
    vsfree_way_r = {c_way_bits{1'b0}};
    for (i = 0; i < p_num_ways; i = i + 1) begin
      if (!tag_rd_victim[i][c_tag_entry_sz-2] && !vsfree_r) begin // not valid = free
        vsfree_r     = 1'b1;
        vsfree_way_r = i[c_way_bits-1:0];
      end
    end
  end

  assign victim_set_has_free = vsfree_r;
  assign victim_free_way     = vsfree_way_r;

  //----------------------------------------------------------------------
  // Victim-set LRU eviction info
  //----------------------------------------------------------------------

  wire [c_tag_entry_sz-1:0] victim_evict_entry = tag_rd_victim[victim_lru_way];
  wire [c_line_bits-1:0]    victim_evict_data_w = data_rd_victim[victim_lru_way];

  assign victim_set_evict_dirty = victim_evict_entry[c_tag_entry_sz-1];
  assign victim_set_evict_line  = victim_evict_data_w;
  assign victim_set_evict_addr  = {victim_evict_entry[c_tag_sz-1:0], vic_set_idx, {c_offset_sz{1'b0}}};

  //----------------------------------------------------------------------
  // CPU word extraction
  //----------------------------------------------------------------------

  wire [c_offset_sz-3:0]  word_idx = req_offset[c_offset_sz-1:2]; //req_offset[3:2]

  assign hit_line      = data_rd_refill[hit_way];
  assign up_resp_rdata = (hit_line[word_idx * p_data_sz +: p_data_sz]) >> (req_offset[1:0] * 8);

  //----------------------------------------------------------------------
  // Cache invalidation at reset
  //----------------------------------------------------------------------

  integer si, wi;
  initial begin
    for (si = 0; si < p_num_sets; si = si + 1)
      for (wi = 0; wi < p_num_ways; wi = wi + 1)
        tag_array[si][wi] = {c_tag_entry_sz{1'b0}};
  end

  always @(posedge clk) begin
    if (reset) begin
      for (si = 0; si < p_num_sets; si = si + 1)
        for (wi = 0; wi < p_num_ways; wi = wi + 1)
          tag_array[si][wi] <= {c_tag_entry_sz{1'b0}};
    end
  end

endmodule

`endif /* CACHE_BASE_CACHE_DPATH_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
