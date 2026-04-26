//=========================================================================
// L2 / L3 Cache  (SWAP upstream and downstream)
//=========================================================================
`ifndef CACHE_BASE_CACHE_V
`define CACHE_BASE_CACHE_V

`include "cache-SwapMsg.vh"
`include "cache-BaseCacheCtrl.v"
`include "cache-BaseCacheDpath.v"

module cache_BaseCache
#(
  parameter p_num_sets  = 512,
  parameter p_num_ways  = 4,
  parameter p_line_sz   = 16,
  parameter p_hit_lat   = 4,
  parameter p_addr_sz   = 32,
  parameter p_data_sz   = 32
)(
  input clk,
  input reset,

  input  [`CACHE_SWAP_REQ_SZ(p_line_sz)-1:0]   up_req_msg,
  input                                          up_req_val,
  output                                         up_req_rdy,

  output [`CACHE_SWAP_RESP_SZ(p_line_sz)-1:0]   up_resp_msg,
  output                                         up_resp_val,
  input                                          up_resp_rdy,

  output [`CACHE_SWAP_REQ_SZ(p_line_sz)-1:0]   dn_req_msg,
  output                                         dn_req_val,
  input                                          dn_req_rdy,

  input  [`CACHE_SWAP_RESP_SZ(p_line_sz)-1:0]  dn_resp_msg,
  input                                          dn_resp_val,
  output                                         dn_resp_rdy
);

  localparam c_line_bits = p_line_sz * 8;
  localparam c_way_bits  = $clog2(p_num_ways > 1 ? p_num_ways : 2);
  localparam c_set_bits  = $clog2(p_num_sets > 1 ? p_num_sets : 2);

  //----------------------------------------------------------------------
  // Unpack upstream SWAP request
  // Layout (MSB→LSB): has_victim | has_refill | victim_addr | refill_addr | victim_data
  //----------------------------------------------------------------------

  wire [c_line_bits-1:0]  up_victim_data  = up_req_msg[c_line_bits-1    : 0];
  wire [p_addr_sz-1:0]    up_refill_addr  = up_req_msg[c_line_bits+31   : c_line_bits];
  wire [p_addr_sz-1:0]    up_victim_addr  = up_req_msg[c_line_bits+63   : c_line_bits+32];
  wire                     up_victim_dirty = up_req_msg[c_line_bits+64];
  wire                     up_has_refill   = up_req_msg[c_line_bits+65];
  wire                     up_has_victim   = up_req_msg[c_line_bits+66];

  //----------------------------------------------------------------------
  // Unpack downstream SWAP response
  //----------------------------------------------------------------------

  wire [c_line_bits-1:0]  dn_refill_data = dn_resp_msg[c_line_bits-1 : 0];
  wire                     dn_has_data    = dn_resp_msg[c_line_bits];

  //----------------------------------------------------------------------
  // Dpath ↔ ctrl wires
  //----------------------------------------------------------------------

  wire                  hit;
  wire [c_way_bits-1:0] hit_way;
  wire                  same_set;

  wire [c_way_bits-1:0] refill_lru_way;
  wire [c_line_bits-1:0] hit_line;
  wire [p_addr_sz-1:0]  refill_evict_addr;
  wire [c_line_bits-1:0] refill_evict_line;
  wire                   refill_evict_dirty;
  wire                   refill_evict_valid;

  wire [c_set_bits-1:0] victim_set_idx;
  wire                   victim_set_has_free;
  wire [c_way_bits-1:0]  victim_free_way;
  wire [c_way_bits-1:0]  victim_lru_way;
  wire [p_addr_sz-1:0]   victim_set_evict_addr;
  wire [c_line_bits-1:0] victim_set_evict_line;
  wire                    victim_set_evict_dirty;

  wire                    refill_tag_wen_en;
  wire                    refill_data_wen_en;
  wire                    refill_invalidate_en;
  wire                    victim_tag_wen_en;
  wire                    victim_data_wen_en;
  wire                    victim_set_evict_en;
  wire                    mark_dirty;
  wire                    lru_update_refill_en;
  wire                    lru_update_victim_en;
  wire                    inplace_swap;

  wire                    ctrl_dn_req_val;
  wire                    ctrl_dn_req_has_victim;
  wire                    ctrl_dn_req_has_refill;
  wire [p_addr_sz-1:0]    ctrl_dn_req_refill_addr;
  wire [p_addr_sz-1:0]    ctrl_dn_req_victim_addr;
  wire                    ctrl_up_req_rdy;
  wire                    ctrl_up_resp_val;
  wire                    ctrl_dn_resp_rdy;
  wire                    ctrl_tag_check;

  //----------------------------------------------------------------------
  // Controller
  //----------------------------------------------------------------------

  cache_BaseCacheCtrl #(
    .p_num_sets (p_num_sets),
    .p_num_ways (p_num_ways),
    .p_line_sz  (p_line_sz),
    .p_addr_sz  (p_addr_sz),
    .p_data_sz  (p_data_sz),
    .p_hit_lat  (p_hit_lat)
  ) ctrl (
    .clk                     (clk),
    .reset                   (reset),

    .up_req_val              (up_req_val),
    .up_req_rdy              (ctrl_up_req_rdy),
    .up_req_has_victim       (up_has_victim),
    .up_req_has_refill       (up_has_refill),
    .up_req_victim_addr      (up_victim_addr),
    .up_req_refill_addr      (up_refill_addr),
    .up_resp_val             (ctrl_up_resp_val),
    .up_resp_rdy             (up_resp_rdy),

    .dn_req_val              (ctrl_dn_req_val),
    .dn_req_rdy              (dn_req_rdy),
    .dn_req_has_victim       (ctrl_dn_req_has_victim),
    .dn_req_has_refill       (ctrl_dn_req_has_refill),
    .dn_req_victim_addr      (ctrl_dn_req_victim_addr),
    .dn_req_refill_addr      (ctrl_dn_req_refill_addr),
    .dn_resp_val             (dn_resp_val),
    .dn_resp_rdy             (ctrl_dn_resp_rdy),
    .dn_resp_has_data        (dn_has_data),

    .refill_tag_wen_en       (refill_tag_wen_en),
    .refill_data_wen_en      (refill_data_wen_en),
    .refill_invalidate_en    (refill_invalidate_en),
    .victim_tag_wen_en       (victim_tag_wen_en),
    .victim_data_wen_en      (victim_data_wen_en),
    .victim_set_evict_en     (victim_set_evict_en),
    .mark_dirty              (mark_dirty),
    .lru_update_refill_en    (lru_update_refill_en),
    .lru_update_victim_en    (lru_update_victim_en),
    .inplace_swap            (inplace_swap),
    .tag_check               (ctrl_tag_check),

    .hit                     (hit),
    .same_set                (same_set),
    .refill_evict_valid      (refill_evict_valid),
    .refill_evict_dirty      (refill_evict_dirty),
    .refill_evict_addr       (refill_evict_addr),
    .victim_set_has_free     (victim_set_has_free),
    .victim_set_evict_dirty  (victim_set_evict_dirty)
  );

  assign up_req_rdy  = ctrl_up_req_rdy;
  assign up_resp_val = ctrl_up_resp_val;

  //----------------------------------------------------------------------
  // One-hot way decoders
  //----------------------------------------------------------------------

  // Refill set: use hit_way on hit (INPLACE_SWAP/INVALIDATE_A),
  // use refill_lru_way on miss (REFILL_WR).
  // The ctrl's refill_tag_wen_en is asserted in both INPLACE_SWAP and REFILL_WR.
  // In INPLACE_SWAP (inplace_swap=1), write the incoming victim into hit_way.
  // In REFILL_WR (inplace_swap=0), write the refill data into refill_lru_way.

  wire [c_way_bits-1:0] refill_write_way = inplace_swap ? hit_way : refill_lru_way;
  wire [c_way_bits-1:0] refill_inv_way   = hit_way; // invalidate always targets hit_way

  // One-hot encoders (behavioural; replace with generate if synthesis requires)
  reg [p_num_ways-1:0] refill_tag_wen_oh;
  reg [p_num_ways-1:0] refill_data_wen_oh;
  reg [p_num_ways-1:0] refill_inv_oh;
  reg [p_num_ways-1:0] victim_tag_wen_oh;
  reg [p_num_ways-1:0] victim_data_wen_oh;
  reg [p_num_ways-1:0] victim_set_evict_oh;

  integer w;
  always @(*) begin
    refill_tag_wen_oh   = {p_num_ways{1'b0}};
    refill_data_wen_oh  = {p_num_ways{1'b0}};
    refill_inv_oh       = {p_num_ways{1'b0}};
    victim_tag_wen_oh   = {p_num_ways{1'b0}};
    victim_data_wen_oh  = {p_num_ways{1'b0}};
    victim_set_evict_oh = {p_num_ways{1'b0}};
    for (w = 0; w < p_num_ways; w = w + 1) begin
      if (refill_tag_wen_en  && w == refill_write_way) refill_tag_wen_oh[w]   = 1'b1;
      if (refill_data_wen_en && w == refill_write_way) refill_data_wen_oh[w]  = 1'b1;
      if (refill_invalidate_en && w == refill_inv_way) refill_inv_oh[w]       = 1'b1;
      if (victim_tag_wen_en  && w == (victim_set_has_free ? victim_free_way : victim_lru_way))
        victim_tag_wen_oh[w]  = 1'b1;
      if (victim_data_wen_en && w == (victim_set_has_free ? victim_free_way : victim_lru_way))
        victim_data_wen_oh[w] = 1'b1;
      if (victim_set_evict_en && w == victim_lru_way)
        victim_set_evict_oh[w] = 1'b1;
    end
  end

  //----------------------------------------------------------------------
  // Data source mux for refill slot
  // inplace_swap=1: write incoming victim data (V takes A's exact slot)
  // inplace_swap=0: write downstream refill data (normal miss path)
  //----------------------------------------------------------------------

  wire [c_line_bits-1:0] refill_line_mux = inplace_swap ? up_victim_data : dn_refill_data;

  //----------------------------------------------------------------------
  // Datapath
  //----------------------------------------------------------------------

  wire [p_data_sz-1:0] up_resp_rdata;

  cache_BaseCacheDpath #(
    .p_num_sets (p_num_sets),
    .p_num_ways (p_num_ways),
    .p_line_sz  (p_line_sz),
    .p_addr_sz  (p_addr_sz),
    .p_data_sz  (p_data_sz)
  ) dpath (
    .clk                    (clk),
    .reset                  (reset),

    .refill_addr            (up_refill_addr),
    .up_req_wdata           ({p_data_sz{1'b0}}),
    .up_req_type            (1'b0),
    .up_req_len             (2'b0),
    .up_resp_rdata          (up_resp_rdata),

    .incoming_victim_addr   (up_victim_addr),
    .incoming_victim_data   (up_victim_data),
    .incoming_victim_dirty  (up_victim_dirty),

    .refill_tag_wen         (refill_tag_wen_oh),
    .refill_data_wen        (refill_data_wen_oh),
    .refill_line            (refill_line_mux),
    .refill_invalidate_way  (refill_inv_oh),
    .mark_dirty             (mark_dirty),
    .inplace_swap           (inplace_swap),
    .store_hit_data_wen     ({p_num_ways{1'b0}}),

    .victim_tag_wen         (victim_tag_wen_oh),
    .victim_data_wen        (victim_data_wen_oh),
    .victim_set_evict_way   (victim_set_evict_oh),

    .lru_update_refill_way  (hit ? hit_way : refill_lru_way),
    .lru_update_refill_en   (lru_update_refill_en),
    .lru_update_victim_way  (victim_set_has_free ? victim_free_way : victim_lru_way),
    .lru_update_victim_en   (lru_update_victim_en),

    .hit                    (hit),
    .hit_way                (hit_way),
    .refill_lru_way         (refill_lru_way),
    .hit_line               (hit_line),
    .refill_evict_addr      (refill_evict_addr),
    .refill_evict_line      (refill_evict_line),
    .refill_evict_dirty     (refill_evict_dirty),
    .refill_evict_valid     (refill_evict_valid),
    .victim_set_idx         (victim_set_idx),
    .same_set               (same_set),
    .victim_set_has_free    (victim_set_has_free),
    .victim_free_way        (victim_free_way),
    .victim_lru_way         (victim_lru_way),
    .victim_set_evict_addr  (victim_set_evict_addr),
    .victim_set_evict_line  (victim_set_evict_line),
    .victim_set_evict_dirty (victim_set_evict_dirty)
  );

  //----------------------------------------------------------------------
  // Pack upstream SWAP response
  // has_data=1 indicates refill data is present (hit or miss with data)
  //----------------------------------------------------------------------

  // On hit: the line was in this cache — use dpath's up_resp_rdata (word)
  // But the upstream level needs the full LINE (to store in L1).
  // Latch hit_line (= data_rd_refill[hit_way]) during TAG_CHECK before it can be
  // overwritten (e.g. by INPLACE_SWAP on the next clock edge).
  // Also track whether the response comes from a hit (latched line) or a miss
  // (dn_refill_data received from downstream).

  reg [c_line_bits-1:0] hit_line_latch;
  reg                   resp_from_hit;

  always @(posedge clk) begin
    if (reset) begin
      hit_line_latch <= {c_line_bits{1'b0}};
      resp_from_hit  <= 1'b0;
    end else begin
      if (ctrl_tag_check) begin
        hit_line_latch <= hit_line;   // always capture; only used when resp_from_hit=1
        resp_from_hit  <= hit;        // 1 on hit, 0 on miss
      end
    end
  end

  // Hit: return the latched line from this cache.
  // Miss: forward dn_refill_data that arrived from downstream.
  wire [c_line_bits-1:0] up_resp_line = resp_from_hit ? hit_line_latch : dn_refill_data;

  assign up_resp_msg = {1'b1, up_resp_line};

  //----------------------------------------------------------------------
  // Pack downstream SWAP request
  // Victim mux:
  //   EVICT_V_REQ (has_refill=0): victim = victim_set_evict (V3)
  //   SWAP_REQ    (has_refill=1): victim = refill_evict     (V2, this cache's own)
  //----------------------------------------------------------------------

  // Downstream victim is always V3 from victim_set_evict_*:
  //   EVICT_V_REQ (hit path, has_refill=0): evicting V3 to make room for V1
  //   SWAP_REQ    (miss path, has_refill=1): V3 piggybacked onto the refill SWAP
  // L2/L3 never evict from the refill set — they don't keep A.
  wire [p_addr_sz-1:0]   dn_victim_addr  = victim_set_evict_addr;
  wire [c_line_bits-1:0] dn_victim_data  = victim_set_evict_line;
  wire                   dn_victim_dirty = victim_set_evict_dirty;

  assign dn_req_msg = {
    ctrl_dn_req_has_victim,
    ctrl_dn_req_has_refill,
    dn_victim_dirty,
    dn_victim_addr,
    ctrl_dn_req_refill_addr,
    dn_victim_data
  };

  assign dn_req_val  = ctrl_dn_req_val;
  assign dn_resp_rdy = ctrl_dn_resp_rdy;

endmodule

`endif /* CACHE_BASE_CACHE_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
