//=========================================================================
// L1 Cache  (VC_MEM upstream, SWAP downstream)
//=========================================================================
`ifndef CACHE_L1_CACHE_V
`define CACHE_L1_CACHE_V

`include "vc-MemReqMsg.v"
`include "vc-MemRespMsg.v"
`include "cache-SwapMsg.vh"
`include "cache-L1CacheCtrl.v"
`include "cache-BaseCacheDpath.v"

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

  output [`CACHE_SWAP_REQ_SZ(p_line_sz)-1:0]             dn_req_msg,
  output                                                  dn_req_val,
  input                                                   dn_req_rdy,

  input  [`CACHE_SWAP_RESP_SZ(p_line_sz)-1:0]            dn_resp_msg,
  input                                                   dn_resp_val,
  output                                                  dn_resp_rdy
);

  localparam c_line_bits   = p_line_sz * 8;
  localparam c_offset_sz   = $clog2(p_line_sz);
  localparam c_way_bits    = $clog2(p_num_ways > 1 ? p_num_ways : 2);

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

  wire                   dn_resp_has_data = dn_resp_msg[c_line_bits];
  wire [c_line_bits-1:0] dn_resp_refill_line = dn_resp_msg[c_line_bits-1:0];

  //----------------------------------------------------------------------
  // Controller
  //----------------------------------------------------------------------

  wire ctrl_up_req_rdy;
  wire ctrl_up_resp_val;
  wire ctrl_dn_req_val;
  wire ctrl_dn_resp_rdy;

  wire                  refill_tag_wen_en;
  wire                  refill_data_wen_en;
  wire                  refill_invalidate;
  wire                  mark_dirty;
  wire                  lru_update_refill_en;

  wire                  ctrl_dn_req_has_victim;
  wire                  ctrl_dn_req_has_refill;
  wire [p_addr_sz-1:0]  ctrl_dn_req_refill_addr;

  wire                  hit;
  wire [c_way_bits-1:0] hit_way;
  wire [c_way_bits-1:0] refill_lru_way;
  wire [p_addr_sz-1:0]  refill_evict_addr;
  wire [c_line_bits-1:0] refill_evict_line;
  wire                  refill_evict_dirty;
  wire                  refill_evict_valid;

  // Latched request inside ctrl; expose needed fields
  wire req_type_lat;
  wire [p_addr_sz-1:0] req_addr_lat;
  wire [p_data_sz-1:0] req_wdata_lat;

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
    .up_req_type          (req_type),
    .up_req_addr          (req_addr),
    .up_req_wdata         (req_wdata),
    .up_resp_val          (ctrl_up_resp_val),

    .req_addr_lat_out     (req_addr_lat),
    .req_wdata_lat_out    (req_wdata_lat),
    .req_type_lat_out     (req_type_lat),

    .dn_req_val           (ctrl_dn_req_val),
    .dn_req_rdy           (dn_req_rdy),
    .dn_req_has_victim    (ctrl_dn_req_has_victim),
    .dn_req_has_refill    (ctrl_dn_req_has_refill),
    .dn_req_victim_addr   (),
    .dn_req_refill_addr   (ctrl_dn_req_refill_addr),
    .dn_resp_val          (dn_resp_val),
    .dn_resp_rdy          (ctrl_dn_resp_rdy),
    .dn_resp_has_data     (dn_resp_has_data),

    .refill_tag_wen_en    (refill_tag_wen_en),
    .refill_data_wen_en   (refill_data_wen_en),
    .refill_invalidate    (refill_invalidate),
    .mark_dirty           (mark_dirty),
    .lru_update_refill_en (lru_update_refill_en),

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
  reg [p_num_ways-1:0] refill_inv_oh;

  integer w;
  always @(*) begin
    refill_tag_wen_oh  = {p_num_ways{1'b0}};
    refill_data_wen_oh = {p_num_ways{1'b0}};
    refill_inv_oh      = {p_num_ways{1'b0}};
    for (w = 0; w < p_num_ways; w = w + 1) begin
      if (refill_tag_wen_en  && w == refill_lru_way) refill_tag_wen_oh[w]  = 1'b1;
      if (refill_data_wen_en && w == refill_lru_way) refill_data_wen_oh[w] = 1'b1;
      if (refill_invalidate  && w == hit_way)         refill_inv_oh[w]      = 1'b1;
    end
  end

  //----------------------------------------------------------------------
  // Latch the hit line for the response (captured at TAG_CHECK when hit)
  // Also latch the refill line when downstream SWAP response arrives.
  //----------------------------------------------------------------------

  reg [c_line_bits-1:0] resp_line_latch;
  reg                   resp_is_hit;

  // up_resp_rdata from dpath: word at req_offset from the hit/refill line.
  wire [p_data_sz-1:0] up_resp_rdata;

  always @(posedge clk) begin
    if (reset) begin
      resp_line_latch <= {c_line_bits{1'b0}};
      resp_is_hit     <= 1'b0;
    end else begin
      // Capture on the cycle we discover a hit (TAG_CHECK cycle output)
      if (hit && ctrl_up_req_rdy == 1'b0) begin
        // hit is combinational; latch while still in TAG_CHECK
        resp_is_hit <= 1'b1;
      end
      // Capture downstream refill line when SWAP response arrives
      if (dn_resp_val && ctrl_dn_resp_rdy) begin
        resp_line_latch <= dn_resp_refill_line;
        resp_is_hit     <= 1'b0;
      end
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
    .clk                    (clk),
    .reset                  (reset),

    .refill_addr            (req_addr_lat),
    .up_req_wdata           (req_wdata_lat),
    .up_req_type            (req_type_lat),
    .up_resp_rdata          (up_resp_rdata),

    .incoming_victim_addr   ({p_addr_sz{1'b0}}),
    .incoming_victim_data   ({c_line_bits{1'b0}}),
    .incoming_victim_dirty  (1'b0),

    .refill_tag_wen         (refill_tag_wen_oh),
    .refill_data_wen        (refill_data_wen_oh),
    .refill_line            (dn_resp_refill_line),
    .refill_invalidate_way  (refill_inv_oh),
    .mark_dirty             (mark_dirty),

    .victim_tag_wen         ({p_num_ways{1'b0}}),
    .victim_data_wen        ({p_num_ways{1'b0}}),
    .victim_set_evict_way   ({p_num_ways{1'b0}}),

    .lru_update_refill_way  (hit_way),
    .lru_update_refill_en   (lru_update_refill_en),
    .lru_update_victim_way  ({c_way_bits{1'b0}}),
    .lru_update_victim_en   (1'b0),

    .hit                    (hit),
    .hit_way                (hit_way),
    .refill_lru_way         (refill_lru_way),
    .hit_line               (),              // unused in L1: word extracted via up_resp_rdata
    .refill_evict_addr      (refill_evict_addr),
    .refill_evict_line      (refill_evict_line),
    .refill_evict_dirty     (refill_evict_dirty),
    .refill_evict_valid     (refill_evict_valid),

    .victim_set_idx         (),
    .same_set               (),
    .victim_set_has_free    (),
    .victim_free_way        (),
    .victim_lru_way         (),
    .victim_set_evict_addr  (),
    .victim_set_evict_line  (),
    .victim_set_evict_dirty ()
  );

  //----------------------------------------------------------------------
  // Pack upstream response (VC_MEM)
  // up_resp_rdata from dpath gives the correct word from whichever line
  // is currently readable (hit line or refill line after REFILL_WR).
  //----------------------------------------------------------------------

  vc_MemRespMsgToBits #(p_data_sz) resp_pack (
    .type (req_type),
    .len  (2'd0),
    .data (up_resp_rdata),
    .bits (up_resp_msg)
  );

  //----------------------------------------------------------------------
  // Pack downstream SWAP request
  //----------------------------------------------------------------------

  assign dn_req_msg = {
    ctrl_dn_req_has_victim,
    ctrl_dn_req_has_refill,
    refill_evict_dirty,        // dirty bit travels with the victim
    refill_evict_addr,
    ctrl_dn_req_refill_addr,
    refill_evict_line
  };

  assign dn_req_val  = ctrl_dn_req_val;
  assign dn_resp_rdy = ctrl_dn_resp_rdy;

endmodule

`endif /* CACHE_L1_CACHE_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
