//=========================================================================
// Inclusive L2 / L3 Cache  (INCL upstream and downstream)
//=========================================================================
`ifndef CACHE_BASE_CACHE_V
`define CACHE_BASE_CACHE_V

`include "inclcache-InclMsg.vh"
`include "inclcache-BaseCacheCtrl.v"
`include "inclcache-BaseCacheDpath.v"

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

  input  [`CACHE_INCL_REQ_SZ(p_line_sz)-1:0]   up_req_msg,
  input                                          up_req_val,
  output                                         up_req_rdy,

  output [`CACHE_INCL_RESP_SZ(p_line_sz)-1:0]   up_resp_msg,
  output                                         up_resp_val,
  input                                          up_resp_rdy,

  output [`CACHE_INCL_REQ_SZ(p_line_sz)-1:0]   dn_req_msg,
  output                                         dn_req_val,
  input                                          dn_req_rdy,

  input  [`CACHE_INCL_RESP_SZ(p_line_sz)-1:0]  dn_resp_msg,
  input                                          dn_resp_val,
  output                                         dn_resp_rdy
);

  localparam c_line_bits = p_line_sz * 8;
  localparam c_way_bits  = $clog2(p_num_ways > 1 ? p_num_ways : 2);

  //----------------------------------------------------------------------
  // Unpack upstream INCL request
  // Layout: has_victim | victim_addr | refill_addr | victim_data
  //----------------------------------------------------------------------

  wire [c_line_bits-1:0]  up_victim_data = up_req_msg[c_line_bits-1    : 0];
  wire [p_addr_sz-1:0]    up_refill_addr = up_req_msg[c_line_bits+31   : c_line_bits];
  wire [p_addr_sz-1:0]    up_victim_addr = up_req_msg[c_line_bits+63   : c_line_bits+32];
  wire                     up_has_victim  = up_req_msg[c_line_bits+64];

  //----------------------------------------------------------------------
  // Unpack downstream INCL response
  //----------------------------------------------------------------------

  wire [c_line_bits-1:0] dn_refill_data = dn_resp_msg[c_line_bits-1:0];

  //----------------------------------------------------------------------
  // Latch upstream request fields on accept (IDLE → TAG_CHECK)
  //----------------------------------------------------------------------

  reg                   lat_has_victim;
  reg [p_addr_sz-1:0]   lat_refill_addr;
  reg [p_addr_sz-1:0]   lat_victim_addr;
  reg [c_line_bits-1:0] lat_victim_data;

  wire ctrl_up_req_rdy;

  always @(posedge clk) begin
    if (reset) begin
      lat_has_victim  <= 1'b0;
      lat_refill_addr <= {p_addr_sz{1'b0}};
      lat_victim_addr <= {p_addr_sz{1'b0}};
      lat_victim_data <= {c_line_bits{1'b0}};
    end else if (up_req_val && ctrl_up_req_rdy) begin
      lat_has_victim  <= up_has_victim;
      lat_refill_addr <= up_refill_addr;
      lat_victim_addr <= up_victim_addr;
      lat_victim_data <= up_victim_data;
    end
  end

  //----------------------------------------------------------------------
  // Dpath ↔ Ctrl wires
  //----------------------------------------------------------------------

  wire                   hit;
  wire [c_way_bits-1:0]  hit_way;
  wire [c_way_bits-1:0]  victim_hit_way;
  wire [c_way_bits-1:0]  refill_lru_way;
  wire [c_line_bits-1:0] hit_line;
  wire [p_addr_sz-1:0]   refill_evict_addr;
  wire [c_line_bits-1:0] refill_evict_line;
  wire                    refill_evict_dirty;
  wire                    refill_evict_valid;

  wire                    refill_tag_wen_en;
  wire                    refill_data_wen_en;
  wire                    mark_dirty;
  wire                    lru_update_refill_en;
  wire                    store_hit_wen_en;
  wire                    victim_tag_wen_en;
  wire                    victim_data_wen_en;
  wire                    tag_check;

  wire                    ctrl_up_resp_val;
  wire                    ctrl_dn_req_val;
  wire                    ctrl_dn_resp_rdy;
  wire                    ctrl_dn_req_has_victim;
  wire [p_addr_sz-1:0]    ctrl_dn_req_refill_addr;

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
    .clk                  (clk),
    .reset                (reset),

    .up_req_val           (up_req_val),
    .up_req_rdy           (ctrl_up_req_rdy),
    .lat_has_victim       (lat_has_victim),
    .lat_refill_addr      (lat_refill_addr),
    .up_resp_val          (ctrl_up_resp_val),
    .up_resp_rdy          (up_resp_rdy),

    .dn_req_val           (ctrl_dn_req_val),
    .dn_req_rdy           (dn_req_rdy),
    .dn_req_has_victim    (ctrl_dn_req_has_victim),
    .dn_req_refill_addr   (ctrl_dn_req_refill_addr),
    .dn_resp_val          (dn_resp_val),
    .dn_resp_rdy          (ctrl_dn_resp_rdy),

    .refill_tag_wen_en    (refill_tag_wen_en),
    .refill_data_wen_en   (refill_data_wen_en),
    .mark_dirty           (mark_dirty),
    .lru_update_refill_en (lru_update_refill_en),
    .store_hit_wen_en     (store_hit_wen_en),
    .victim_tag_wen_en    (victim_tag_wen_en),
    .victim_data_wen_en   (victim_data_wen_en),
    .tag_check            (tag_check),

    .hit                  (hit),
    .refill_evict_valid   (refill_evict_valid),
    .refill_evict_dirty   (refill_evict_dirty),
    .req_type_lat         (1'b0)   // L2/L3: no CPU stores handled here
  );

  assign up_req_rdy  = ctrl_up_req_rdy;
  assign up_resp_val = ctrl_up_resp_val;

  //----------------------------------------------------------------------
  // One-hot decoders for dpath write enables
  //----------------------------------------------------------------------

  reg [p_num_ways-1:0] refill_tag_wen_oh;
  reg [p_num_ways-1:0] refill_data_wen_oh;
  reg [p_num_ways-1:0] store_hit_data_wen_oh;
  reg [p_num_ways-1:0] victim_tag_wen_oh;
  reg [p_num_ways-1:0] victim_data_wen_oh;

  integer w;
  always @(*) begin
    refill_tag_wen_oh     = {p_num_ways{1'b0}};
    refill_data_wen_oh    = {p_num_ways{1'b0}};
    store_hit_data_wen_oh = {p_num_ways{1'b0}};
    victim_tag_wen_oh     = {p_num_ways{1'b0}};
    victim_data_wen_oh    = {p_num_ways{1'b0}};
    for (w = 0; w < p_num_ways; w = w + 1) begin
      if (refill_tag_wen_en  && w == refill_lru_way)  refill_tag_wen_oh[w]     = 1'b1;
      if (refill_data_wen_en && w == refill_lru_way)  refill_data_wen_oh[w]    = 1'b1;
      if (store_hit_wen_en   && w == hit_way)          store_hit_data_wen_oh[w] = 1'b1;
      if (victim_tag_wen_en  && w == victim_hit_way)   victim_tag_wen_oh[w]     = 1'b1;
      if (victim_data_wen_en && w == victim_hit_way)   victim_data_wen_oh[w]    = 1'b1;
    end
  end

  //----------------------------------------------------------------------
  // Latch the response line
  // On TAG_CHECK when hit: capture hit_line.
  // On DN_WAIT when response arrives: capture refill data from downstream.
  //----------------------------------------------------------------------

  reg [c_line_bits-1:0] resp_line;

  always @(posedge clk) begin
    if (reset) begin
      resp_line <= {c_line_bits{1'b0}};
    end else begin
      if (tag_check && hit)
        resp_line <= hit_line;
      if (dn_resp_val && ctrl_dn_resp_rdy)
        resp_line <= dn_refill_data;
    end
  end

  //----------------------------------------------------------------------
  // Datapath
  //----------------------------------------------------------------------

  cache_BaseCacheDpath #(
    .p_num_sets (p_num_sets),
    .p_num_ways (p_num_ways),
    .p_line_sz  (p_line_sz),
    .p_addr_sz  (p_addr_sz),
    .p_data_sz  (p_data_sz)
  ) dpath (
    .clk                   (clk),
    .reset                 (reset),

    .refill_addr           (lat_refill_addr),
    .up_req_wdata          ({p_data_sz{1'b0}}),  // stores not handled at L2/L3
    .up_req_type           (1'b0),
    .up_req_len            (2'b0),
    .up_resp_rdata         (),                   // word response unused at L2/L3

    .incoming_victim_addr  (lat_victim_addr),
    .incoming_victim_data  (lat_victim_data),

    .refill_tag_wen        (refill_tag_wen_oh),
    .refill_data_wen       (refill_data_wen_oh),
    .refill_line           (dn_refill_data),
    .refill_preserve_dirty (1'b0),
    .mark_dirty            (mark_dirty),
    .store_hit_data_wen    (store_hit_data_wen_oh),

    .victim_tag_wen        (victim_tag_wen_oh),
    .victim_data_wen       (victim_data_wen_oh),

    .lru_update_refill_way (hit_way),
    .lru_update_refill_en  (lru_update_refill_en),

    .hit                   (hit),
    .hit_way               (hit_way),
    .refill_lru_way        (refill_lru_way),
    .hit_line              (hit_line),
    .refill_evict_addr     (refill_evict_addr),
    .refill_evict_line     (refill_evict_line),
    .refill_evict_dirty    (refill_evict_dirty),
    .refill_evict_valid    (refill_evict_valid),
    .victim_hit_way        (victim_hit_way)
  );

  //----------------------------------------------------------------------
  // Pack upstream response
  // Layout: has_data | refill_data  (has_data always 1)
  //----------------------------------------------------------------------

  assign up_resp_msg = {1'b1, resp_line};

  //----------------------------------------------------------------------
  // Pack downstream INCL request
  // Layout: has_victim | victim_addr | refill_addr | victim_data
  //----------------------------------------------------------------------

  assign dn_req_msg = {
    ctrl_dn_req_has_victim,  // dirty eviction from refill_set (if any)
    refill_evict_addr,       // evicted line's address
    ctrl_dn_req_refill_addr, // address to fetch from downstream
    refill_evict_line        // evicted line's data
  };

  assign dn_req_val  = ctrl_dn_req_val;
  assign dn_resp_rdy = ctrl_dn_resp_rdy;

endmodule

`endif /* CACHE_BASE_CACHE_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
