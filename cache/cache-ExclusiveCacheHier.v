//=========================================================================
// Exclusive Three-Level Cache Hierarchy for riscvlong  (SWAP protocol)
//=========================================================================
//
// Topology
// --------
//
//   riscv_Core
//     imem (VC_MEM)     dmem (VC_MEM)
//         |                 |
//      cache_L1Cache     cache_L1Cache
//      (L1I, read-only)  (L1D, read-write)
//         |                 |
//     SWAP req          SWAP req
//         \               /
//       cache_Arbiter2to1        ← round-robin serialiser
//               |
//           SWAP req
//               |
//         cache_BaseCache  (L2, unified)
//               |
//           SWAP req
//               |
//         cache_BaseCache  (L3, unified)
//               |
//           SWAP req
//               |
//       cache_L3MemAdapter       ← decomposes SWAP into word-sized VC_MEM
//               |
//   vc_TestDualPortRandDelayMem  (port 0 only)
//
// Exclusive-cache invariant
// -------------------------
//   Any valid line lives in EXACTLY ONE level at any time.
//   SWAP messages carry both the outgoing victim and the refill address,
//   allowing the receiving level to perform an in-place slot swap when
//   the two addresses happen to index the same set (avoiding unnecessary
//   downstream evictions).  The set-index mismatch case is handled by
//   the BaseCacheCtrl FSM (EVICT_V → PLACE_V → SWAP_REQ path).
//
// Val/Rdy CPU stalling
// --------------------
//   L1I/L1D caches deassert imemreq_rdy / dmemreq_rdy while a miss is
//   being serviced.  The CPU holds its request stable until rdy is
//   re-asserted.  up_resp_val is a one-cycle pulse; no back-pressure from
//   the CPU side (riscvlong-Core has no memresp_rdy output).

`ifndef CACHE_EXCLUSIVE_CACHE_HIER_V
`define CACHE_EXCLUSIVE_CACHE_HIER_V

`include "vc-MemReqMsg.v"
`include "vc-MemRespMsg.v"
`include "cache-SwapMsg.vh"
`include "cache-L1Cache.v"
`include "cache-BaseCache.v"
`include "cache-Arbiter2to1.v"
`include "cache-L3MemAdapter.v"

module cache_ExclusiveCacheHier
#(
  // L1 Instruction Cache
  parameter p_l1i_num_sets  = 256,
  parameter p_l1i_num_ways  = 1,
  parameter p_l1i_line_sz   = 16,
  parameter p_l1i_hit_lat   = 1,

  // L1 Data Cache
  parameter p_l1d_num_sets  = 256,
  parameter p_l1d_num_ways  = 1,
  parameter p_l1d_line_sz   = 16,
  parameter p_l1d_hit_lat   = 1,

  // L2 Unified Cache
  parameter p_l2_num_sets   = 512,
  parameter p_l2_num_ways   = 4,
  parameter p_l2_line_sz    = 16,   // must equal l1x_line_sz for clean SWAP protocol
  parameter p_l2_hit_lat    = 4,

  // L3 Unified Cache
  parameter p_l3_num_sets   = 2048,
  parameter p_l3_num_ways   = 8,
  parameter p_l3_line_sz    = 16,   // must equal l2_line_sz
  parameter p_l3_hit_lat    = 10,

  // Fixed widths (match riscvlong-Core and DualPortRandDelayMem)
  parameter p_addr_sz       = 32,
  parameter p_data_sz       = 32
)(
  input clk,
  input reset,

  //======================================================================
  // CPU-facing interface  (connect directly to riscvlong-Core ports)
  //======================================================================

  input  [`VC_MEM_REQ_MSG_SZ(p_addr_sz,p_data_sz)-1:0]  imemreq_msg,
  input                                                   imemreq_val,
  output                                                  imemreq_rdy,  // stalls pipeline
  output [`VC_MEM_RESP_MSG_SZ(p_data_sz)-1:0]            imemresp_msg,
  output                                                  imemresp_val,

  input  [`VC_MEM_REQ_MSG_SZ(p_addr_sz,p_data_sz)-1:0]  dmemreq_msg,
  input                                                   dmemreq_val,
  output                                                  dmemreq_rdy,  // stalls pipeline
  output [`VC_MEM_RESP_MSG_SZ(p_data_sz)-1:0]            dmemresp_msg,
  output                                                  dmemresp_val,

  //======================================================================
  // Memory-facing interface  (connect to DualPortRandDelayMem port 0)
  //======================================================================
  // Port 1 of DualPortRandDelayMem is unused; tie its inputs:
  //   .memreq1_val(1'b0), .memresp1_rdy(1'b1)

  output [`VC_MEM_REQ_MSG_SZ(p_addr_sz,p_data_sz)-1:0]  memreq_msg,
  output                                                  memreq_val,
  input                                                   memreq_rdy,

  input  [`VC_MEM_RESP_MSG_SZ(p_data_sz)-1:0]            memresp_msg,
  input                                                   memresp_val,
  output                                                  memresp_rdy
);

  //======================================================================
  // Internal SWAP wire declarations
  // All line sizes are equal (enforced by parameter constraints above).
  // Using p_l2_line_sz as the canonical inter-cache line size.
  //======================================================================

  localparam LINE_SZ = p_l2_line_sz;

  // ---- L1I → Arbiter port 0 ------------------------------------------
  wire [`CACHE_SWAP_REQ_SZ(LINE_SZ)-1:0]   l1i_dn_req_msg;
  wire                                       l1i_dn_req_val;
  wire                                       l1i_dn_req_rdy;
  wire [`CACHE_SWAP_RESP_SZ(LINE_SZ)-1:0]  l1i_dn_resp_msg;
  wire                                       l1i_dn_resp_val;
  wire                                       l1i_dn_resp_rdy;

  // ---- L1D → Arbiter port 1 ------------------------------------------
  wire [`CACHE_SWAP_REQ_SZ(LINE_SZ)-1:0]   l1d_dn_req_msg;
  wire                                       l1d_dn_req_val;
  wire                                       l1d_dn_req_rdy;
  wire [`CACHE_SWAP_RESP_SZ(LINE_SZ)-1:0]  l1d_dn_resp_msg;
  wire                                       l1d_dn_resp_val;
  wire                                       l1d_dn_resp_rdy;

  // ---- Arbiter → L2 upstream -----------------------------------------
  wire [`CACHE_SWAP_REQ_SZ(LINE_SZ)-1:0]   arb_l2_req_msg;
  wire                                       arb_l2_req_val;
  wire                                       arb_l2_req_rdy;
  wire [`CACHE_SWAP_RESP_SZ(LINE_SZ)-1:0]  arb_l2_resp_msg;
  wire                                       arb_l2_resp_val;
  wire                                       arb_l2_resp_rdy;

  // ---- L2 downstream → L3 upstream -----------------------------------
  wire [`CACHE_SWAP_REQ_SZ(LINE_SZ)-1:0]   l2_dn_req_msg;
  wire                                       l2_dn_req_val;
  wire                                       l2_dn_req_rdy;
  wire [`CACHE_SWAP_RESP_SZ(LINE_SZ)-1:0]  l2_dn_resp_msg;
  wire                                       l2_dn_resp_val;
  wire                                       l2_dn_resp_rdy;

  // ---- L3 downstream → L3MemAdapter ----------------------------------
  wire [`CACHE_SWAP_REQ_SZ(LINE_SZ)-1:0]   l3_dn_req_msg;
  wire                                       l3_dn_req_val;
  wire                                       l3_dn_req_rdy;
  wire [`CACHE_SWAP_RESP_SZ(LINE_SZ)-1:0]  l3_dn_resp_msg;
  wire                                       l3_dn_resp_val;
  wire                                       l3_dn_resp_rdy;

  //======================================================================
  // L1 Instruction Cache  (read-only; CPU sends only loads / ifeches)
  //======================================================================

  cache_L1Cache #(
    .p_num_sets (p_l1i_num_sets),
    .p_num_ways (p_l1i_num_ways),
    .p_line_sz  (p_l1i_line_sz),
    .p_hit_lat  (p_l1i_hit_lat),
    .p_addr_sz  (p_addr_sz),
    .p_data_sz  (p_data_sz)
  ) l1i (
    .clk          (clk),
    .reset        (reset),
    .up_req_msg   (imemreq_msg),
    .up_req_val   (imemreq_val),
    .up_req_rdy   (imemreq_rdy),
    .up_resp_msg  (imemresp_msg),
    .up_resp_val  (imemresp_val),
    .dn_req_msg   (l1i_dn_req_msg),
    .dn_req_val   (l1i_dn_req_val),
    .dn_req_rdy   (l1i_dn_req_rdy),
    .dn_resp_msg  (l1i_dn_resp_msg),
    .dn_resp_val  (l1i_dn_resp_val),
    .dn_resp_rdy  (l1i_dn_resp_rdy)
  );

  //======================================================================
  // L1 Data Cache  (read-write)
  //======================================================================

  cache_L1Cache #(
    .p_num_sets (p_l1d_num_sets),
    .p_num_ways (p_l1d_num_ways),
    .p_line_sz  (p_l1d_line_sz),
    .p_hit_lat  (p_l1d_hit_lat),
    .p_addr_sz  (p_addr_sz),
    .p_data_sz  (p_data_sz)
  ) l1d (
    .clk          (clk),
    .reset        (reset),
    .up_req_msg   (dmemreq_msg),
    .up_req_val   (dmemreq_val),
    .up_req_rdy   (dmemreq_rdy),
    .up_resp_msg  (dmemresp_msg),
    .up_resp_val  (dmemresp_val),
    .dn_req_msg   (l1d_dn_req_msg),
    .dn_req_val   (l1d_dn_req_val),
    .dn_req_rdy   (l1d_dn_req_rdy),
    .dn_resp_msg  (l1d_dn_resp_msg),
    .dn_resp_val  (l1d_dn_resp_val),
    .dn_resp_rdy  (l1d_dn_resp_rdy)
  );

  //======================================================================
  // L1 → L2 Arbiter
  //======================================================================

  cache_Arbiter2to1 #(
    .p_line_sz (LINE_SZ),
    .p_addr_sz (p_addr_sz)
  ) l1_to_l2_arb (
    .clk         (clk),
    .reset       (reset),
    // Port 0: L1I
    .req0_msg    (l1i_dn_req_msg),
    .req0_val    (l1i_dn_req_val),
    .req0_rdy    (l1i_dn_req_rdy),
    .resp0_msg   (l1i_dn_resp_msg),
    .resp0_val   (l1i_dn_resp_val),
    .resp0_rdy   (l1i_dn_resp_rdy),
    // Port 1: L1D
    .req1_msg    (l1d_dn_req_msg),
    .req1_val    (l1d_dn_req_val),
    .req1_rdy    (l1d_dn_req_rdy),
    .resp1_msg   (l1d_dn_resp_msg),
    .resp1_val   (l1d_dn_resp_val),
    .resp1_rdy   (l1d_dn_resp_rdy),
    // Downstream: L2 upstream
    .dn_req_msg  (arb_l2_req_msg),
    .dn_req_val  (arb_l2_req_val),
    .dn_req_rdy  (arb_l2_req_rdy),
    .dn_resp_msg (arb_l2_resp_msg),
    .dn_resp_val (arb_l2_resp_val),
    .dn_resp_rdy (arb_l2_resp_rdy)
  );

  //======================================================================
  // L2 Unified Cache
  //======================================================================

  cache_BaseCache #(
    .p_num_sets (p_l2_num_sets),
    .p_num_ways (p_l2_num_ways),
    .p_line_sz  (p_l2_line_sz),
    .p_hit_lat  (p_l2_hit_lat),
    .p_addr_sz  (p_addr_sz),
    .p_data_sz  (p_data_sz)
  ) l2 (
    .clk          (clk),
    .reset        (reset),
    .up_req_msg   (arb_l2_req_msg),
    .up_req_val   (arb_l2_req_val),
    .up_req_rdy   (arb_l2_req_rdy),
    .up_resp_msg  (arb_l2_resp_msg),
    .up_resp_val  (arb_l2_resp_val),
    .up_resp_rdy  (1'b1),            // L1 always ready when stalled
    .dn_req_msg   (l2_dn_req_msg),
    .dn_req_val   (l2_dn_req_val),
    .dn_req_rdy   (l2_dn_req_rdy),
    .dn_resp_msg  (l2_dn_resp_msg),
    .dn_resp_val  (l2_dn_resp_val),
    .dn_resp_rdy  (l2_dn_resp_rdy)
  );

  //======================================================================
  // L3 Unified Cache
  //======================================================================

  cache_BaseCache #(
    .p_num_sets (p_l3_num_sets),
    .p_num_ways (p_l3_num_ways),
    .p_line_sz  (p_l3_line_sz),
    .p_hit_lat  (p_l3_hit_lat),
    .p_addr_sz  (p_addr_sz),
    .p_data_sz  (p_data_sz)
  ) l3 (
    .clk          (clk),
    .reset        (reset),
    .up_req_msg   (l2_dn_req_msg),
    .up_req_val   (l2_dn_req_val),
    .up_req_rdy   (l2_dn_req_rdy),
    .up_resp_msg  (l2_dn_resp_msg),
    .up_resp_val  (l2_dn_resp_val),
    .up_resp_rdy  (1'b1),            // L2 always ready when stalled
    .dn_req_msg   (l3_dn_req_msg),
    .dn_req_val   (l3_dn_req_val),
    .dn_req_rdy   (l3_dn_req_rdy),
    .dn_resp_msg  (l3_dn_resp_msg),
    .dn_resp_val  (l3_dn_resp_val),
    .dn_resp_rdy  (l3_dn_resp_rdy)
  );

  //======================================================================
  // L3 ↔ Main Memory Adapter
  //======================================================================

  cache_L3MemAdapter #(
    .p_line_sz (p_l3_line_sz),
    .p_addr_sz (p_addr_sz),
    .p_data_sz (p_data_sz)
  ) l3_mem_adapter (
    .clk         (clk),
    .reset       (reset),
    .dn_req_msg  (l3_dn_req_msg),
    .dn_req_val  (l3_dn_req_val),
    .dn_req_rdy  (l3_dn_req_rdy),
    .dn_resp_msg (l3_dn_resp_msg),
    .dn_resp_val (l3_dn_resp_val),
    .dn_resp_rdy (l3_dn_resp_rdy),
    .memreq_msg  (memreq_msg),
    .memreq_val  (memreq_val),
    .memreq_rdy  (memreq_rdy),
    .memresp_msg (memresp_msg),
    .memresp_val (memresp_val),
    .memresp_rdy (memresp_rdy)
  );

endmodule

`endif /* CACHE_EXCLUSIVE_CACHE_HIER_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
