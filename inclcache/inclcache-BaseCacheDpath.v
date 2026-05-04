// Inclusive cache datapath (no victim-set staging, no inplace_swap; adds victim_hit_way).
// Two simultaneous set lookups: req_set_idx (refill) and vic_set_idx (victim writeback).

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

  // Primary address being looked up or filled
  input  [p_addr_sz-1:0]    refill_addr,
  // Store data / type / len (used for write-allocate and store-hit paths)
  input  [p_data_sz-1:0]    up_req_wdata,
  input                      up_req_type,
  input  [1:0]               up_req_len,    // 0=word, 1=byte, 2=halfword
  output [p_data_sz-1:0]     up_resp_rdata,

  // Incoming victim from upstream (L1 dirty eviction written into L2/L3)
  input  [p_addr_sz-1:0]    incoming_victim_addr,
  input  [p_line_sz*8-1:0]  incoming_victim_data,

  // Refill-set write enables (one-hot over p_num_ways)
  input  [p_num_ways-1:0]   refill_tag_wen,
  input  [p_num_ways-1:0]   refill_data_wen,
  input  [p_line_sz*8-1:0]  refill_line,       // data arriving from downstream
  input                      refill_preserve_dirty,
  input                      mark_dirty,        // mark refill slot dirty (write-allocate)
  input  [p_num_ways-1:0]   store_hit_data_wen,

  // Victim-set write enables (one-hot over p_num_ways)
  // Used to update the L2/L3 copy of the incoming victim.
  input  [p_num_ways-1:0]   victim_tag_wen,    // set dirty=1 for victim's slot
  input  [p_num_ways-1:0]   victim_data_wen,   // overwrite data for victim's slot

  // LRU update (refill set only; victim-set LRU not updated for simplicity)
  input [$clog2(p_num_ways > 1 ? p_num_ways : 2)-1:0] lru_update_refill_way,
  input                                                 lru_update_refill_en,

  // Hit detection (refill set)
  output                                                hit,
  output [$clog2(p_num_ways > 1 ? p_num_ways : 2)-1:0] hit_way,

  // LRU eviction candidate (refill set)
  output [$clog2(p_num_ways > 1 ? p_num_ways : 2)-1:0] refill_lru_way,

  // Hit line data (data_array[req_set_idx][hit_way])
  output [p_line_sz*8-1:0]  hit_line,

  // Eviction info from refill set's LRU slot
  output [p_addr_sz-1:0]    refill_evict_addr,
  output [p_line_sz*8-1:0]  refill_evict_line,
  output                     refill_evict_dirty,
  output                     refill_evict_valid,

  // Which way in vic_set_idx the incoming victim is stored
  output [$clog2(p_num_ways > 1 ? p_num_ways : 2)-1:0] victim_hit_way
);

  localparam c_offset_sz      = $clog2(p_line_sz);
  localparam c_idx_sz         = $clog2(p_num_sets > 1 ? p_num_sets : 2);
  localparam c_tag_sz         = p_addr_sz - c_idx_sz - c_offset_sz;
  localparam c_line_bits      = p_line_sz * 8;
  localparam c_way_bits       = $clog2(p_num_ways > 1 ? p_num_ways : 2);
  localparam c_words_per_line = p_line_sz / (p_data_sz / 8);

  // Tag entry: dirty(1) | valid(1) | tag(c_tag_sz)
  localparam c_tag_entry_sz = 1 + 1 + c_tag_sz;
  // Address decomposition
  wire [c_idx_sz-1:0]   req_set_idx = refill_addr[c_offset_sz+c_idx_sz-1 : c_offset_sz];
  wire [c_tag_sz-1:0]   req_tag     = refill_addr[p_addr_sz-1 : c_offset_sz+c_idx_sz];
  wire [c_offset_sz-1:0] req_offset = refill_addr[c_offset_sz-1:0];

  wire [c_idx_sz-1:0]   vic_set_idx = incoming_victim_addr[c_offset_sz+c_idx_sz-1 : c_offset_sz];
  wire [c_tag_sz-1:0]   vic_tag     = incoming_victim_addr[p_addr_sz-1 : c_offset_sz+c_idx_sz];
  // Tag SRAM
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

  integer i;
  always @(posedge clk) begin
    for (i = 0; i < p_num_ways; i = i + 1) begin
      // Refill-set: write new tag (normal miss refill)
      if (refill_tag_wen[i])
        tag_array[req_set_idx][i] <= {mark_dirty || refill_preserve_dirty, 1'b1, req_tag};
      // Refill-set: mark dirty on store hit
      if (store_hit_data_wen[i])
        tag_array[req_set_idx][i][c_tag_entry_sz-1] <= 1'b1;
      // Victim-set: update victim's slot — dirty=1 (victim is always dirty in inclusive)
      if (victim_tag_wen[i])
        tag_array[vic_set_idx][i] <= {1'b1, 1'b1, vic_tag};
    end
  end
  // Data SRAM
  reg [c_line_bits-1:0] data_array [0:p_num_sets-1][0:p_num_ways-1];

  wire [c_line_bits-1:0] data_rd_refill [0:p_num_ways-1];

  generate
    for (gw = 0; gw < p_num_ways; gw = gw + 1) begin : g_data_rd
      assign data_rd_refill[gw] = data_array[req_set_idx][gw];
    end
  endgenerate

  // Pre-compute write-allocate merged line combinatorially (avoids overlapping NBAs)
  reg [c_line_bits-1:0] wa_merged_line;
  always @(*) begin
    wa_merged_line = refill_line;
    if (mark_dirty) begin
      if (up_req_len == 2'd1)
        wa_merged_line[req_offset[c_offset_sz-1:2]*p_data_sz + req_offset[1:0]*8 +: 8]  = up_req_wdata[7:0];
      else if (up_req_len == 2'd2)
        wa_merged_line[req_offset[c_offset_sz-1:2]*p_data_sz + req_offset[1:0]*8 +: 16] = up_req_wdata[15:0];
      else
        wa_merged_line[req_offset[c_offset_sz-1:2]*p_data_sz +: p_data_sz] = up_req_wdata;
    end
  end

  integer j;
  always @(posedge clk) begin
    for (j = 0; j < p_num_ways; j = j + 1) begin
      if (refill_data_wen[j])
        data_array[req_set_idx][j] <= wa_merged_line;
      if (victim_data_wen[j])
        data_array[vic_set_idx][j] <= incoming_victim_data;
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
  // Hit detection (refill set)
  reg                   hit_r;
  reg [c_way_bits-1:0]  hit_way_r;

  always @(*) begin
    hit_r     = 1'b0;
    hit_way_r = {c_way_bits{1'b0}};
    for (i = 0; i < p_num_ways; i = i + 1) begin
      if (tag_rd_refill[i][c_tag_entry_sz-2] &&
          tag_rd_refill[i][c_tag_sz-1:0] == req_tag) begin
        hit_r     = 1'b1;
        hit_way_r = i[c_way_bits-1:0];
      end
    end
  end

  assign hit     = hit_r;
  assign hit_way = hit_way_r;
  // Victim-set lookup: find which way holds the incoming victim
  reg [c_way_bits-1:0] victim_hit_way_r;

  always @(*) begin
    victim_hit_way_r = {c_way_bits{1'b0}};
    for (i = 0; i < p_num_ways; i = i + 1) begin
      if (tag_rd_victim[i][c_tag_entry_sz-2] &&
          tag_rd_victim[i][c_tag_sz-1:0] == vic_tag)
        victim_hit_way_r = i[c_way_bits-1:0];
    end
  end

  assign victim_hit_way = victim_hit_way_r;
  // Tree Pseudo-LRU (refill set only)
  localparam c_plru_levels = $clog2(p_num_ways > 1 ? p_num_ways : 2);
  localparam c_plru_bits   = (p_num_ways > 1) ? (p_num_ways - 1) : 1;

  reg [c_plru_bits-1:0] plru [0:p_num_sets-1];

  reg [c_way_bits-1:0] refill_lru_way_r;
  integer plru_r_node, plru_r_lvl;

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

  assign refill_lru_way = (p_num_ways > 1) ? refill_lru_way_r : {c_way_bits{1'b0}};

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
            plru[req_set_idx][plru_upd_node] <= 1'b1;
            plru_upd_node = 2 * plru_upd_node + 1;
          end else begin
            plru[req_set_idx][plru_upd_node] <= 1'b0;
            plru_upd_node = 2 * plru_upd_node + 2;
          end
        end
      end
    end
  end
  // Refill-set eviction info (LRU way)
  wire [c_tag_entry_sz-1:0] refill_evict_entry = tag_rd_refill[refill_lru_way];

  assign refill_evict_valid = refill_evict_entry[c_tag_entry_sz-2];
  assign refill_evict_dirty = refill_evict_entry[c_tag_entry_sz-1];
  assign refill_evict_line  = data_rd_refill[refill_lru_way];
  assign refill_evict_addr  = {refill_evict_entry[c_tag_sz-1:0], req_set_idx, {c_offset_sz{1'b0}}};
  // CPU word extraction
  wire [c_offset_sz-3:0] word_idx = req_offset[c_offset_sz-1:2];

  assign hit_line      = data_rd_refill[hit_way];
  assign up_resp_rdata = (hit_line[word_idx * p_data_sz +: p_data_sz]) >> (req_offset[1:0] * 8);
  // Cache initialisation at reset
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
