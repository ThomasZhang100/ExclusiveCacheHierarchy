//=========================================================================
// L1 Cache  (VC_MEM upstream, INCL downstream)
//=========================================================================
`ifndef CACHE_L1_CACHE_V
`define CACHE_L1_CACHE_V

`include "vc-MemReqMsg.v"
`include "vc-MemRespMsg.v"
`include "./cache-InclMsg.vh"
`include "./cache-L1CacheCtrl.v"
`include "./cache-BaseCacheDpath.v"

module cache_L1Cache
#(
  parameter p_num_sets  = 256,
  parameter p_num_ways  = 1,
  parameter p_line_sz   = 16,
  parameter p_hit_lat   = 1,
  parameter p_addr_sz   = 32,
  parameter p_data_sz   = 32
)(
  input clk,
  input reset,

  input  [`VC_MEM_REQ_MSG_SZ(p_addr_sz,p_data_sz)-1:0]  up_req_msg,
  input                                                   up_req_val,
  output                                                  up_req_rdy,

  output [`VC_MEM_RESP_MSG_SZ(p_data_sz)-1:0]            up_resp_msg,
  output                                                  up_resp_val,

  output [`CACHE_INCL_REQ_SZ(p_line_sz)-1:0]             dn_req_msg,
  output                                                  dn_req_val,
  input                                                   dn_req_rdy,

  input  [`CACHE_INCL_RESP_SZ(p_line_sz)-1:0]            dn_resp_msg,
  input                                                   dn_resp_val,
  output                                                  dn_resp_rdy
);

  localparam c_line_bits = p_line_sz * 8;
  localparam c_offset_sz = $clog2(p_line_sz);
  localparam c_way_bits  = $clog2(p_num_ways > 1 ? p_num_ways : 2);

  //----------------------------------------------------------------------
  // Unpack CPU request
  //----------------------------------------------------------------------

  wire                   req_type;
  wire [p_addr_sz-1:0]   req_addr;
  wire [1:0]             req_len;
  wire [p_data_sz-1:0]   req_wdata;

  vc_MemReqMsgFromBits #(p_addr_sz, p_data_sz) req_unpack (
    .bits (up_req_msg),
    .type (req_type),
    .addr (req_addr),
    .len  (req_len),
    .data (req_wdata)
  );

  //----------------------------------------------------------------------
  // Downstream response fields
  //----------------------------------------------------------------------

  wire [c_line_bits-1:0] dn_resp_refill_line = dn_resp_msg[c_line_bits-1:0];

  //----------------------------------------------------------------------
  // Latched request fields
  //----------------------------------------------------------------------

  reg                  req_type_lat;
  reg [1:0]            req_len_lat;
  reg [p_addr_sz-1:0]  req_addr_lat;
  reg [p_data_sz-1:0]  req_wdata_lat;

  wire ctrl_up_req_rdy;

  always @(posedge clk) begin
    if (reset) begin
      req_type_lat  <= 1'b0;
      req_len_lat   <= 2'b0;
      req_addr_lat  <= {p_addr_sz{1'b0}};
      req_wdata_lat <= {p_data_sz{1'b0}};
    end else if (up_req_val && ctrl_up_req_rdy) begin
      req_type_lat  <= req_type;
      req_len_lat   <= req_len;
      req_addr_lat  <= req_addr;
      req_wdata_lat <= req_wdata;
    end
  end

  //----------------------------------------------------------------------
  // Controller
  //----------------------------------------------------------------------

  wire ctrl_up_resp_val;
  wire ctrl_dn_req_val;
  wire ctrl_dn_resp_rdy;
  wire ctrl_dn_req_has_victim;
  wire [p_addr_sz-1:0] ctrl_dn_req_refill_addr;

  wire                   refill_tag_wen_en;
  wire                   refill_data_wen_en;
  wire                   mark_dirty;
  wire                   lru_update_refill_en;
  wire                   store_hit_wen_en;

  wire                   hit;
  wire [c_way_bits-1:0]  hit_way;
  wire [c_way_bits-1:0]  refill_lru_way;
  wire [p_addr_sz-1:0]   refill_evict_addr;
  wire [c_line_bits-1:0] refill_evict_line;
  wire                   refill_evict_dirty;
  wire                   refill_evict_valid;

  cache_L1CacheCtrl #(
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
    .req_addr_lat         (req_addr_lat),
    .req_type_lat         (req_type_lat),
    .up_resp_val          (ctrl_up_resp_val),

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

    .hit                  (hit),
    .refill_evict_valid   (refill_evict_valid),
    .refill_evict_dirty   (refill_evict_dirty)
  );

  assign up_req_rdy  = ctrl_up_req_rdy;
  assign up_resp_val = ctrl_up_resp_val;

  //----------------------------------------------------------------------
  // One-hot decoders for dpath write enables
  //----------------------------------------------------------------------

  reg [p_num_ways-1:0] refill_tag_wen_oh;
  reg [p_num_ways-1:0] refill_data_wen_oh;
  reg [p_num_ways-1:0] store_hit_data_wen_oh;

  integer w;
  always @(*) begin
    refill_tag_wen_oh     = {p_num_ways{1'b0}};
    refill_data_wen_oh    = {p_num_ways{1'b0}};
    store_hit_data_wen_oh = {p_num_ways{1'b0}};
    for (w = 0; w < p_num_ways; w = w + 1) begin
      if (refill_tag_wen_en  && w == refill_lru_way) refill_tag_wen_oh[w]     = 1'b1;
      if (refill_data_wen_en && w == refill_lru_way) refill_data_wen_oh[w]    = 1'b1;
      if (store_hit_wen_en   && w == hit_way)        store_hit_data_wen_oh[w] = 1'b1;
    end
  end

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
    .clk                   (clk),
    .reset                 (reset),

    .refill_addr           (req_addr_lat),
    .up_req_wdata          (req_wdata_lat),
    .up_req_type           (req_type_lat),
    .up_req_len            (req_len_lat),
    .up_resp_rdata         (up_resp_rdata),

    // L1 never receives an incoming victim from above
    .incoming_victim_addr  ({p_addr_sz{1'b0}}),
    .incoming_victim_data  ({c_line_bits{1'b0}}),

    .refill_tag_wen        (refill_tag_wen_oh),
    .refill_data_wen       (refill_data_wen_oh),
    .refill_line           (dn_resp_refill_line),
    .mark_dirty            (mark_dirty),
    .store_hit_data_wen    (store_hit_data_wen_oh),

    // L1 never writes an incoming victim
    .victim_tag_wen        ({p_num_ways{1'b0}}),
    .victim_data_wen       ({p_num_ways{1'b0}}),

    .lru_update_refill_way (hit_way),
    .lru_update_refill_en  (lru_update_refill_en),

    .hit                   (hit),
    .hit_way               (hit_way),
    .refill_lru_way        (refill_lru_way),
    .hit_line              (),
    .refill_evict_addr     (refill_evict_addr),
    .refill_evict_line     (refill_evict_line),
    .refill_evict_dirty    (refill_evict_dirty),
    .refill_evict_valid    (refill_evict_valid),
    .victim_hit_way        ()
  );

  //----------------------------------------------------------------------
  // Pack upstream response (VC_MEM)
  //----------------------------------------------------------------------

  vc_MemRespMsgToBits #(p_data_sz) resp_pack (
    .type (req_type),
    .len  (2'd0),
    .data (up_resp_rdata),
    .bits (up_resp_msg)
  );

  //----------------------------------------------------------------------
  // Pack downstream INCL request
  // Layout: has_victim | victim_addr | refill_addr | victim_data
  //----------------------------------------------------------------------

  assign dn_req_msg = {
    ctrl_dn_req_has_victim,   // 1: dirty victim being donated
    refill_evict_addr,        // victim_addr (from dpath)
    ctrl_dn_req_refill_addr,  // refill_addr (line-aligned)
    refill_evict_line         // victim_data (from dpath)
  };

  assign dn_req_val  = ctrl_dn_req_val;
  assign dn_resp_rdy = ctrl_dn_resp_rdy;

endmodule

`endif /* CACHE_L1_CACHE_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
