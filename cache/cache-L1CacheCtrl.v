//=========================================================================
// L1 Cache Controller
//=========================================================================
// Handles the L1-specific upstream protocol: the CPU sends plain word-sized
// VC_MEM requests (addr only, no victim).  The L1 controller is responsible
// for constructing a SWAP message to send downstream, packaging:
//
//   has_victim  = refill_evict_valid   (is the LRU line actually occupied?)
//   has_refill  = 1                    (always — L1 always needs the data)
//   victim_addr = refill_evict_addr    (from dpath — reconstructed from stored tag)
//   victim_data = refill_evict_line    (from dpath — full line data)
//   refill_addr = line-aligned CPU request address
//
// Val/Rdy stalling
// ----------------
//   up_req_rdy is deasserted while the FSM is not in STATE_IDLE.
//   The CPU must hold its request stable until up_req_rdy is re-asserted.
//   up_resp_val is asserted for one cycle when data is ready; because the
//   CPU is stalled at that point no up_resp_rdy handshake is needed.

`ifndef CACHE_L1_CACHE_CTRL_V
`define CACHE_L1_CACHE_CTRL_V

module cache_L1CacheCtrl
#(
  parameter p_num_sets  = 256,
  parameter p_num_ways  = 1,
  parameter p_line_sz   = 16,
  parameter p_addr_sz   = 32,
  parameter p_data_sz   = 32,
  parameter p_hit_lat   = 1
)(
  input clk,
  input reset,

  //----------------------------------------------------------------------
  // Upstream: CPU-facing word-sized VC_MEM interface
  //----------------------------------------------------------------------

  input                   up_req_val,
  output reg              up_req_rdy,   // deasserted to stall CPU pipeline

  input                   up_req_type,  // 0=read, 1=write (store)
  input  [p_addr_sz-1:0]  up_req_addr,
  input  [p_data_sz-1:0]  up_req_wdata,

  output reg              up_resp_val,  // one-cycle pulse when data is ready

  // Expose latched request address for stable word extraction in dpath
  output reg [p_addr_sz-1:0] req_addr_lat_out,
  output reg [p_addr_sz-1:0] req_wdata_lat_out,
  output reg [p_addr_sz-1:0] req_type_lat_out,

  //----------------------------------------------------------------------
  // Downstream: SWAP message interface (to L2 via arbiter)
  // See cache-SwapMsg.vh for message layout.
  //----------------------------------------------------------------------

  output reg              dn_req_val,
  input                   dn_req_rdy,

  // Assembled SWAP request fields (packed into message by enclosing module)
  output reg              dn_req_has_victim,
  output reg              dn_req_has_refill,   // always 1 for L1
  output reg [p_addr_sz-1:0] dn_req_victim_addr,
  output reg [p_addr_sz-1:0] dn_req_refill_addr,
  // victim_data is driven directly from dpath.refill_evict_line (no register needed)

  input                   dn_resp_val,
  output reg              dn_resp_rdy,
  input                   dn_resp_has_data,    // should always be 1 for L1 refills

  //----------------------------------------------------------------------
  // Dpath control outputs
  //----------------------------------------------------------------------

  output reg              refill_tag_wen_en,    // enable tag write for refill slot
  output reg              refill_data_wen_en,   // enable data write for refill slot
  output reg              refill_invalidate,    // (unused in L1 miss path; for completeness)
  output reg              mark_dirty,           // write-allocate: store after refill
  output reg              lru_update_refill_en, // update LRU after hit or refill

  //----------------------------------------------------------------------
  // Dpath status inputs
  //----------------------------------------------------------------------

  input                   hit,
  input                   refill_evict_valid,   // victim is occupied (need has_victim=1)
  input                   refill_evict_dirty    // victim is dirty (informational for L1)
);

  //----------------------------------------------------------------------
  // FSM state encoding
  //----------------------------------------------------------------------

  localparam STATE_IDLE       = 3'd0;
  localparam STATE_TAG_CHECK  = 3'd1; // tag SRAM read done; evaluate hit/miss
  localparam STATE_HIT_WAIT   = 3'd2; // count down extra hit latency cycles
  localparam STATE_SWAP_REQ   = 3'd3; // send SWAP request downstream
  localparam STATE_SWAP_WAIT  = 3'd4; // waiting for SWAP response (refill data)
  localparam STATE_REFILL_WR  = 3'd5; // write refill line into cache arrays
  localparam STATE_RESP       = 3'd6; // assert up_resp_val for one cycle

  reg [2:0] state_reg, state_next;

  //----------------------------------------------------------------------
  // Hit-latency counter
  //----------------------------------------------------------------------

  localparam c_hit_cnt_sz = $clog2(p_hit_lat > 1 ? p_hit_lat : 2);
  reg [c_hit_cnt_sz-1:0] hit_lat_cnt;

  //----------------------------------------------------------------------
  // Latched request (held stable while the miss is being served)
  //----------------------------------------------------------------------

  reg                  req_type_lat;
  reg [p_addr_sz-1:0]  req_addr_lat;
  reg [p_data_sz-1:0]  req_wdata_lat;

  assign req_addr_lat_out = req_addr_lat;
  assign req_wdata_lat_out = req_wdata_lat;
  assign req_type_lat_out = req_type_lat;

  localparam c_offset_sz = $clog2(p_line_sz);

  always @(posedge clk) begin
    if (reset) begin
      req_type_lat  <= 1'b0;
      req_addr_lat  <= {p_addr_sz{1'b0}};
      req_wdata_lat <= {p_data_sz{1'b0}};
    end else if (up_req_val && up_req_rdy) begin
      req_type_lat  <= up_req_type;
      req_addr_lat  <= up_req_addr;
      req_wdata_lat <= up_req_wdata;
    end
  end

  //----------------------------------------------------------------------
  // State register
  //----------------------------------------------------------------------

  always @(posedge clk) begin
    if (reset) state_reg <= STATE_IDLE;
    else       state_reg <= state_next;
  end

  //----------------------------------------------------------------------
  // Hit-latency counter update
  //----------------------------------------------------------------------

  always @(posedge clk) begin
    if (reset) begin
      hit_lat_cnt <= {c_hit_cnt_sz{1'b0}};
    end else begin
      if (state_reg == STATE_TAG_CHECK && state_next == STATE_HIT_WAIT)
        hit_lat_cnt <= p_hit_lat - 2; // TAG_CHECK already consumed one cycle
      else if (state_reg == STATE_HIT_WAIT && hit_lat_cnt != {c_hit_cnt_sz{1'b0}})
        hit_lat_cnt <= hit_lat_cnt - 1'b1;
    end
  end

  //----------------------------------------------------------------------
  // Next-state logic
  //----------------------------------------------------------------------

  always @(*) begin
    state_next = state_reg;
    case (state_reg)

      STATE_IDLE: begin
        if (up_req_val)
          state_next = STATE_TAG_CHECK;
      end

      STATE_TAG_CHECK: begin
        if (hit)
          state_next = (p_hit_lat <= 1) ? STATE_RESP : STATE_HIT_WAIT;
        else
          state_next = STATE_SWAP_REQ;
      end

      STATE_HIT_WAIT: begin
        if (hit_lat_cnt == {c_hit_cnt_sz{1'b0}})
          state_next = STATE_RESP;
      end

      STATE_SWAP_REQ: begin
        // Wait until downstream accepts the SWAP request.
        if (dn_req_rdy)
          state_next = STATE_SWAP_WAIT;
      end

      STATE_SWAP_WAIT: begin
        // Wait for downstream to return the refill line.
        if (dn_resp_val)
          state_next = STATE_REFILL_WR;
      end

      STATE_REFILL_WR: begin
        // Write refill data into tag + data arrays (1 cycle SRAM write latency).
        state_next = STATE_RESP;
      end

      STATE_RESP: begin
        state_next = STATE_IDLE;
      end

      default: state_next = STATE_IDLE;

    endcase
  end

  //----------------------------------------------------------------------
  // Output logic
  //----------------------------------------------------------------------

  always @(*) begin
    // Safe defaults
    up_req_rdy           = 1'b0;
    up_resp_val          = 1'b0;
    dn_req_val           = 1'b0;
    dn_req_has_victim    = 1'b0;
    dn_req_has_refill    = 1'b1;     // L1 always needs the refill data back
    dn_req_victim_addr   = {p_addr_sz{1'b0}};
    dn_req_refill_addr   = {p_addr_sz{1'b0}};
    dn_resp_rdy          = 1'b0;
    refill_tag_wen_en    = 1'b0;
    refill_data_wen_en   = 1'b0;
    refill_invalidate    = 1'b0;
    mark_dirty           = 1'b0;
    lru_update_refill_en = 1'b0;

    case (state_reg)

      STATE_IDLE: begin
        up_req_rdy = 1'b1;
      end

      STATE_TAG_CHECK: begin
        // SRAM read in progress; nothing to drive.
      end

      STATE_HIT_WAIT: begin
        // Counter runs in sequential block; no combinational outputs needed.
      end

      STATE_SWAP_REQ: begin
        // Construct the SWAP message and assert dn_req_val.
        // victim info comes from the dpath (refill_evict_*) which is
        // already latched from the tag/data SRAM read during TAG_CHECK.
        dn_req_val         = 1'b1;
        dn_req_has_victim  = refill_evict_valid; // include victim if L1 slot was occupied
        dn_req_has_refill  = 1'b1;
        // Line-align the request address (zero out byte offset bits):
        dn_req_refill_addr = {req_addr_lat[p_addr_sz-1:c_offset_sz], {c_offset_sz{1'b0}}};
        // victim_addr from dpath.refill_evict_addr (wired in enclosing module)
        dn_req_victim_addr = {p_addr_sz{1'b0}}; // driven from dpath in cache-L1Cache.v
        // victim_data (refill_evict_line) wired directly from dpath in enclosing module
      end

      STATE_SWAP_WAIT: begin
        dn_resp_rdy = 1'b1;
      end

      STATE_REFILL_WR: begin
        refill_tag_wen_en    = 1'b1;
        refill_data_wen_en   = 1'b1;
        mark_dirty           = req_type_lat; // write-allocate: mark dirty if it was a store
        lru_update_refill_en = 1'b1;
      end

      STATE_RESP: begin
        up_resp_val          = 1'b1;
        lru_update_refill_en = 1'b1; // also update LRU on hit response
      end

      default: begin
        up_req_rdy = 1'b0;
      end

    endcase
  end

endmodule

`endif /* CACHE_L1_CACHE_CTRL_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
