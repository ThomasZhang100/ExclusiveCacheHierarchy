//=========================================================================
// riscvlong Simulation with Exclusive L1/L2/L3 Cache Hierarchy
//=========================================================================
// Drop-in replacement for riscvlong-randdelay-sim.v that inserts the
// three-level exclusive cache hierarchy between the processor and the
// dual-port random-delay main memory.
//
// Connection diagram:
//
//   riscv_Core
//      |  imem{req,resp}   dmem{req,resp}
//      |        \               /
//      |   cache_ExclusiveCacheHier
//      |              |
//      |       cache  memreq / memresp
//      |              |
//      |   vc_TestDualPortRandDelayMem
//      |           (port 0 only)
//
// Port 1 of the memory is unused; its request input is tied low.

`include "riscvlong-Core.v"
`include "vc-TestDualPortRandDelayMem.v"
`include "../cache/cache-ExclusiveCacheHier.v"

module riscv_cache_sim;

  //----------------------------------------------------------------------
  // Clock and reset
  //----------------------------------------------------------------------

  reg clk   = 1'b0;
  reg reset = 1'b1;

  always #5 clk = ~clk;

  wire [31:0] status;

  //----------------------------------------------------------------------
  // Processor ↔ Cache wires  (word-sized VC_MEM messages)
  //----------------------------------------------------------------------

  wire [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] imemreq_msg;
  wire                                  imemreq_val;
  wire                                  imemreq_rdy;   // from cache (stall)
  wire [`VC_MEM_RESP_MSG_SZ(32)-1:0]   imemresp_msg;
  wire                                  imemresp_val;

  wire [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] dmemreq_msg;
  wire                                  dmemreq_val;
  wire                                  dmemreq_rdy;   // from cache (stall)
  wire [`VC_MEM_RESP_MSG_SZ(32)-1:0]   dmemresp_msg;
  wire                                  dmemresp_val;

  //----------------------------------------------------------------------
  // Cache ↔ Main-memory wires  (word-sized VC_MEM messages, port 0)
  //----------------------------------------------------------------------

  wire [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] cache_memreq_msg;
  wire                                  cache_memreq_val;
  wire                                  cache_memreq_rdy;
  wire [`VC_MEM_RESP_MSG_SZ(32)-1:0]   cache_memresp_msg;
  wire                                  cache_memresp_val;
  wire                                  cache_memresp_rdy;

  //----------------------------------------------------------------------
  // Staggered resets (memory → cache → processor)
  //----------------------------------------------------------------------
  // Memory must be reset first so it is ready before the cache hierarchy
  // and processor start accessing it.

  reg reset_mem;
  reg reset_cache;
  reg reset_proc;

  always @(posedge clk) begin
    reset_mem   <= reset;
    reset_cache <= reset_mem;
    reset_proc  <= reset_cache;
  end

  //----------------------------------------------------------------------
  // riscv_Core
  //----------------------------------------------------------------------

  riscv_Core proc (
    .clk          (clk),
    .reset        (reset_proc),

    // Instruction memory
    .imemreq_msg  (imemreq_msg),
    .imemreq_val  (imemreq_val),
    .imemreq_rdy  (imemreq_rdy),   // ← driven by cache (stalls pipeline)
    .imemresp_msg (imemresp_msg),
    .imemresp_val (imemresp_val),

    // Data memory
    .dmemreq_msg  (dmemreq_msg),
    .dmemreq_val  (dmemreq_val),
    .dmemreq_rdy  (dmemreq_rdy),   // ← driven by cache (stalls pipeline)
    .dmemresp_msg (dmemresp_msg),
    .dmemresp_val (dmemresp_val),

    .csr_status   (status)
  );

  //----------------------------------------------------------------------
  // Exclusive Cache Hierarchy
  //----------------------------------------------------------------------
  // Parameters below are the defaults; adjust as needed.

  cache_ExclusiveCacheHier #(
    // L1I
    .p_l1i_num_sets  (256),
    .p_l1i_num_ways  (1),
    .p_l1i_line_sz   (16),
    .p_l1i_hit_lat   (1),
    // L1D
    .p_l1d_num_sets  (256),
    .p_l1d_num_ways  (1),
    .p_l1d_line_sz   (16),
    .p_l1d_hit_lat   (1),
    // L2
    .p_l2_num_sets   (512),
    .p_l2_num_ways   (4),
    .p_l2_line_sz    (16),
    .p_l2_hit_lat    (4),
    // L3
    .p_l3_num_sets   (2048),
    .p_l3_num_ways   (8),
    .p_l3_line_sz    (16),
    .p_l3_hit_lat    (10)
  ) cache_hier (
    .clk            (clk),
    .reset          (reset_cache),

    // CPU-facing
    .imemreq_msg    (imemreq_msg),
    .imemreq_val    (imemreq_val),
    .imemreq_rdy    (imemreq_rdy),
    .imemresp_msg   (imemresp_msg),
    .imemresp_val   (imemresp_val),

    .dmemreq_msg    (dmemreq_msg),
    .dmemreq_val    (dmemreq_val),
    .dmemreq_rdy    (dmemreq_rdy),
    .dmemresp_msg   (dmemresp_msg),
    .dmemresp_val   (dmemresp_val),

    // Memory-facing (port 0)
    .memreq_msg     (cache_memreq_msg),
    .memreq_val     (cache_memreq_val),
    .memreq_rdy     (cache_memreq_rdy),
    .memresp_msg    (cache_memresp_msg),
    .memresp_val    (cache_memresp_val),
    .memresp_rdy    (cache_memresp_rdy)
  );

  //----------------------------------------------------------------------
  // Main Memory  (vc_TestDualPortRandDelayMem)
  //----------------------------------------------------------------------
  // Only port 0 is used by the cache hierarchy.
  // Port 1 request is tied low; port 1 response ready is tied high.

  vc_TestDualPortRandDelayMem #(
    .p_mem_sz    (1 << 20),  // 1 MB
    .p_addr_sz   (32),
    .p_data_sz   (32),
    .p_max_delay (4)
  ) mem (
    .clk          (clk),
    .reset        (reset_mem),

    // Port 0: used by cache hierarchy
    .memreq0_val  (cache_memreq_val),
    .memreq0_rdy  (cache_memreq_rdy),
    .memreq0_msg  (cache_memreq_msg),
    .memresp0_val (cache_memresp_val),
    .memresp0_rdy (cache_memresp_rdy),
    .memresp0_msg (cache_memresp_msg),

    // Port 1: unused
    .memreq1_val  (1'b0),
    .memreq1_rdy  (),
    .memreq1_msg  ({`VC_MEM_REQ_MSG_SZ(32,32){1'b0}}),
    .memresp1_val (),
    .memresp1_rdy (1'b1),
    .memresp1_msg ()
  );

  //----------------------------------------------------------------------
  // Simulation control  (identical to riscvlong-randdelay-sim.v)
  //----------------------------------------------------------------------

  integer fh;
  reg [1023:0] exe_filename;
  reg [1023:0] vcd_filename;
  reg   [31:0] max_cycles;
  reg          verbose;
  reg          stats;
  reg          vcd;
  reg    [1:0] disasm;

  initial begin

    if ($value$plusargs("exe=%s", exe_filename)) begin
      fh = $fopen(exe_filename, "r");
      if (!fh) begin
        $display("\n ERROR: Could not open vmh file (%s)! \n", exe_filename);
        $finish;
      end
      $fclose(fh);
      $readmemh(exe_filename, mem.mem.m);
    end else begin
      $display("\n ERROR: No executable specified! (use +exe=<filename>) \n");
      $finish;
    end

    if (!$value$plusargs("max-cycles=%d", max_cycles))
      max_cycles = 100000;

    if (!$value$plusargs("stats=%d", stats)) begin
      if (!$value$plusargs("verbose=%d", verbose))
        verbose = 1'b0;
      proc.ctrl.stats_en = 1'b0;
    end else begin
      verbose = 1'b1;
      proc.ctrl.stats_en = 1'b1;
    end

    if ($value$plusargs("vcd=%d", vcd)) begin
      vcd_filename = {exe_filename[983:32], "-cache.vcd"};
      $dumpfile(vcd_filename);
      $dumpvars;
    end

    if (!$value$plusargs("disasm=%d", disasm))
      disasm = 2'b0;

    #5  reset = 1'b1;
    #80 reset = 1'b0;  // extra cycles for 3-stage staggered reset
  end

  //----------------------------------------------------------------------
  // Stop on status change
  //----------------------------------------------------------------------

  real ipc;

  always @(*) begin
    if (!reset && (status != 0)) begin
      if (status == 1'b1)
        $display("*** PASSED ***");
      if (status > 1'b1)
        $display("*** FAILED *** (status = %d)", status);
      if (verbose == 1'b1) begin
        ipc = proc.ctrl.num_inst / $itor(proc.ctrl.num_cycles);
        $display("--------------------------------------------");
        $display(" STATS                                      ");
        $display("--------------------------------------------");
        $display(" status     = %d", status);
        $display(" num_cycles = %d", proc.ctrl.num_cycles);
        $display(" num_inst   = %d", proc.ctrl.num_inst);
        $display(" ipc        = %f", ipc);
      end
      #20 $finish;
    end
  end

  //----------------------------------------------------------------------
  // Timeout
  //----------------------------------------------------------------------

  reg [31:0] cycle_count = 32'b0;

  always @(posedge clk)
    cycle_count = cycle_count + 1'b1;

  always @(*) begin
    if (cycle_count > max_cycles) begin
      #20;
      $display("*** FAILED *** (timeout)");
      $finish;
    end
  end

endmodule

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
