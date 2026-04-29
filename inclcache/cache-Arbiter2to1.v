//=========================================================================
// 2-to-1 Inclusive Request Arbiter
//=========================================================================
// Arbitrates between two upstream INCL requestors (L1I and L1D misses)
// for a single downstream port (L2 cache).  Uses round-robin selection
// and locks the grant for the duration of a transaction (req accepted →
// resp received).
//
// Message widths use the inclusive format (cache-InclMsg.vh):
//   req  msg: `CACHE_INCL_REQ_SZ(p_line_sz)  bits
//   resp msg: `CACHE_INCL_RESP_SZ(p_line_sz) bits

`ifndef CACHE_ARBITER_2TO1_V
`define CACHE_ARBITER_2TO1_V

`include "./cache-InclMsg.vh"

module cache_Arbiter2to1
#(
  parameter p_line_sz = 16,
  parameter p_addr_sz = 32
)(
  input clk,
  input reset,

  //----------------------------------------------------------------------
  // Port 0 upstream  (L1I miss requests)
  //----------------------------------------------------------------------

  input  [`CACHE_INCL_REQ_SZ(p_line_sz)-1:0]   req0_msg,
  input                                          req0_val,
  output                                         req0_rdy,

  output [`CACHE_INCL_RESP_SZ(p_line_sz)-1:0]  resp0_msg,
  output                                         resp0_val,
  input                                          resp0_rdy,

  //----------------------------------------------------------------------
  // Port 1 upstream  (L1D miss requests)
  //----------------------------------------------------------------------

  input  [`CACHE_INCL_REQ_SZ(p_line_sz)-1:0]   req1_msg,
  input                                          req1_val,
  output                                         req1_rdy,

  output [`CACHE_INCL_RESP_SZ(p_line_sz)-1:0]  resp1_msg,
  output                                         resp1_val,
  input                                          resp1_rdy,

  //----------------------------------------------------------------------
  // Downstream  (to L2 cache upstream interface)
  //----------------------------------------------------------------------

  output [`CACHE_INCL_REQ_SZ(p_line_sz)-1:0]   dn_req_msg,
  output                                         dn_req_val,
  input                                          dn_req_rdy,

  input  [`CACHE_INCL_RESP_SZ(p_line_sz)-1:0]  dn_resp_msg,
  input                                          dn_resp_val,
  output                                         dn_resp_rdy
);

  //----------------------------------------------------------------------
  // Grant selection (round-robin)
  //----------------------------------------------------------------------

  reg grant_reg;
  reg lock_active;
  reg lock_grant;

  wire free_grant_sel = (req0_val && req1_val) ? ~grant_reg :
                         req1_val              ? 1'b1        :
                                                 1'b0;

  wire effective_grant = lock_active ? lock_grant : free_grant_sel;

  //----------------------------------------------------------------------
  // Request mux → downstream
  //----------------------------------------------------------------------

  assign dn_req_msg = (effective_grant == 1'b0) ? req0_msg : req1_msg;
  assign dn_req_val = (effective_grant == 1'b0) ? req0_val : req1_val;

  //----------------------------------------------------------------------
  // Ready demux → upstream ports
  //----------------------------------------------------------------------

  assign req0_rdy = (effective_grant == 1'b0) ? dn_req_rdy : 1'b0;
  assign req1_rdy = (effective_grant == 1'b1) ? dn_req_rdy : 1'b0;

  //----------------------------------------------------------------------
  // Response demux → upstream ports
  //----------------------------------------------------------------------

  assign resp0_msg = dn_resp_msg;
  assign resp1_msg = dn_resp_msg;

  assign resp0_val = dn_resp_val & (effective_grant == 1'b0);
  assign resp1_val = dn_resp_val & (effective_grant == 1'b1);

  assign dn_resp_rdy = (effective_grant == 1'b0) ? resp0_rdy : resp1_rdy;

  //----------------------------------------------------------------------
  // Lock and round-robin update (sequential)
  //----------------------------------------------------------------------

  always @(posedge clk) begin
    if (reset) begin
      grant_reg   <= 1'b0;
      lock_active <= 1'b0;
      lock_grant  <= 1'b0;
    end else begin
      if (!lock_active && dn_req_val && dn_req_rdy) begin
        lock_active <= 1'b1;
        lock_grant  <= effective_grant;
        grant_reg   <= effective_grant;
      end
      if (lock_active && dn_resp_val && dn_resp_rdy) begin
        lock_active <= 1'b0;
      end
    end
  end

endmodule

`endif /* CACHE_ARBITER_2TO1_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
