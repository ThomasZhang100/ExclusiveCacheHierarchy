//=========================================================================
// L3 Cache ↔ Main Memory Adapter  (SWAP → word-sized VC_MEM)
//=========================================================================
// Bridges L3's line-sized SWAP downstream interface to the word-sized
// (32-bit data) VC_MEM interface of vc_TestDualPortRandDelayMem.
//
// A single SWAP request from L3 is decomposed into c_words_per_line
// sequential word-sized memory transactions:
//
//   has_refill=1, has_victim=0  →  c_words_per_line READ  requests
//   has_refill=0, has_victim=1  →  c_words_per_line WRITE requests  (pure eviction)
//   has_refill=1, has_victim=1  →  c_words_per_line WRITEs (victim first)
//                                   then c_words_per_line READs  (refill)
//
// Latency note
// ------------
// In real DRAM, only the FIRST word of a burst incurs the full DRAM
// latency (row activation + CAS); subsequent words arrive at bus speed.
// The DualPortRandDelayMem applies an independent random delay per word
// request, which does NOT model this.  A more accurate adapter would issue
// a single address and let a burst controller deliver the remaining words.
// This limitation is noted here; the scaffold matches the available memory
// interface rather than real DRAM timing.

`ifndef CACHE_L3_MEM_ADAPTER_V
`define CACHE_L3_MEM_ADAPTER_V

`include "vc-MemReqMsg.v"
`include "vc-MemRespMsg.v"
`include "cache-SwapMsg.vh"

module cache_L3MemAdapter
#(
  parameter p_line_sz  = 16,   // bytes; must be a multiple of (p_data_sz/8)
  parameter p_addr_sz  = 32,
  parameter p_data_sz  = 32
)(
  input clk,
  input reset,

  //----------------------------------------------------------------------
  // SWAP interface facing L3 downstream
  //----------------------------------------------------------------------

  input  [`CACHE_SWAP_REQ_SZ(p_line_sz)-1:0]   dn_req_msg,
  input                                          dn_req_val,
  output                                         dn_req_rdy,

  output [`CACHE_SWAP_RESP_SZ(p_line_sz)-1:0]  dn_resp_msg,
  output                                         dn_resp_val,
  input                                          dn_resp_rdy,

  //----------------------------------------------------------------------
  // Word-sized VC_MEM interface facing DualPortRandDelayMem (port 0)
  //----------------------------------------------------------------------

  output [`VC_MEM_REQ_MSG_SZ(p_addr_sz,p_data_sz)-1:0]  memreq_msg,
  output                                                  memreq_val,
  input                                                   memreq_rdy,

  input  [`VC_MEM_RESP_MSG_SZ(p_data_sz)-1:0]            memresp_msg,
  input                                                   memresp_val,
  output                                                  memresp_rdy
);

  //----------------------------------------------------------------------
  // Derived constants
  //----------------------------------------------------------------------

  localparam c_line_bits      = p_line_sz * 8;
  localparam c_bytes_per_word = p_data_sz / 8;
  localparam c_words_per_line = p_line_sz / c_bytes_per_word; // e.g. 16/4 = 4
  localparam c_word_cnt_sz    = $clog2(c_words_per_line + 1);

  //----------------------------------------------------------------------
  // Unpack incoming SWAP request
  //----------------------------------------------------------------------

  wire [c_line_bits-1:0]  req_victim_data  = dn_req_msg[c_line_bits-1    : 0];
  wire [p_addr_sz-1:0]    req_refill_addr  = dn_req_msg[c_line_bits+31   : c_line_bits];
  wire [p_addr_sz-1:0]    req_victim_addr  = dn_req_msg[c_line_bits+63   : c_line_bits+32];
  wire                     req_victim_dirty = dn_req_msg[c_line_bits+64];
  wire                     req_has_refill   = dn_req_msg[c_line_bits+65];
  wire                     req_has_victim   = dn_req_msg[c_line_bits+66];

  // Unpack VC_MEM response data field
  // VC_MEM_RESP layout (MSB→LSB): type(1) | len(2) | data(p_data_sz)
  wire [p_data_sz-1:0] memresp_data = memresp_msg[p_data_sz-1:0];

  //----------------------------------------------------------------------
  // FSM state encoding
  //----------------------------------------------------------------------

  localparam STATE_IDLE         = 3'd0; // waiting for SWAP from L3
  localparam STATE_WRITE_REQ    = 3'd1; // issuing word WRITE requests (victim eviction)
  localparam STATE_WRITE_RESP   = 3'd2; // collecting write acks
  localparam STATE_READ_REQ     = 3'd3; // issuing word READ requests (refill)
  localparam STATE_READ_RESP    = 3'd4; // collecting read responses into line_buf
  localparam STATE_DONE         = 3'd5; // drive dn_resp_val for one cycle

  reg [2:0] state_reg, state_next;

  //----------------------------------------------------------------------
  // Word counter and line buffer
  //----------------------------------------------------------------------

  reg [c_word_cnt_sz-1:0] word_cnt;   // request counter (address generation)
  reg [c_word_cnt_sz-1:0] resp_cnt;  // response counter (line_buf assembly)
  reg [c_line_bits-1:0]   line_buf;  // assembled from READ responses

  //----------------------------------------------------------------------
  // Latched SWAP request fields
  //----------------------------------------------------------------------

  reg                   lat_has_refill;
  reg                   lat_has_victim;
  reg                   lat_victim_dirty;
  reg [p_addr_sz-1:0]   lat_refill_addr;
  reg [p_addr_sz-1:0]   lat_victim_addr;
  reg [c_line_bits-1:0] lat_victim_data;

  //----------------------------------------------------------------------
  // State register
  //----------------------------------------------------------------------

  always @(posedge clk) begin
    if (reset) state_reg <= STATE_IDLE;
    else       state_reg <= state_next;
  end

  //----------------------------------------------------------------------
  // Next-state logic
  //----------------------------------------------------------------------

  always @(*) begin
    state_next = state_reg;
    case (state_reg)

      STATE_IDLE: begin
        if (dn_req_val) begin
          // Only write to memory if the victim is dirty; clean lines are discarded
          // since main memory already holds the correct copy.
          if (req_has_victim && req_victim_dirty)
            state_next = STATE_WRITE_REQ;
          else if (req_has_refill)
            state_next = STATE_READ_REQ;
          else
            state_next = STATE_DONE; // clean pure eviction: just ack
        end
      end

      STATE_WRITE_REQ: begin
        // Issue one WRITE per cycle; advance when downstream accepts.
        if (memreq_rdy && word_cnt == c_words_per_line-1)
          state_next = STATE_WRITE_RESP;
      end

      STATE_WRITE_RESP: begin
        // Collect c_words_per_line write acks.
        if (memresp_val && word_cnt == c_words_per_line-1) begin
          state_next = lat_has_refill ? STATE_READ_REQ : STATE_DONE;
        end
      end

      STATE_READ_REQ: begin
        // Issue one READ per cycle.
        if (memreq_rdy && word_cnt == c_words_per_line-1)
          state_next = STATE_READ_RESP;
      end

      STATE_READ_RESP: begin
        // Collect remaining words; also handles the case where all responses
        // already arrived during STATE_READ_REQ (resp_cnt == c_words_per_line).
        if (resp_cnt >= c_words_per_line ||
            (resp_cnt == c_words_per_line-1 && memresp_val))
          state_next = STATE_DONE;
      end

      STATE_DONE: begin
        // Hold dn_resp_val until L3 accepts (dn_resp_rdy).
        if (dn_resp_rdy)
          state_next = STATE_IDLE;
      end

      default: state_next = STATE_IDLE;

    endcase
  end

  //----------------------------------------------------------------------
  // Current word address computation
  //----------------------------------------------------------------------
  // On WRITEs: offset from lat_victim_addr.
  // On READs:  offset from lat_refill_addr.

  wire [p_addr_sz-1:0] cur_base_addr =
    (state_reg == STATE_WRITE_REQ || state_reg == STATE_WRITE_RESP)
      ? lat_victim_addr : lat_refill_addr;

  wire [p_addr_sz-1:0] cur_word_addr =
    cur_base_addr + {{(p_addr_sz-c_word_cnt_sz-2){1'b0}}, word_cnt, 2'b00};

  //----------------------------------------------------------------------
  // Current word data (slice from latched victim line for writes)
  //----------------------------------------------------------------------

  wire [p_data_sz-1:0] cur_word_data = lat_victim_data[word_cnt * p_data_sz +: p_data_sz];

  //----------------------------------------------------------------------
  // Memory request packing
  //----------------------------------------------------------------------

  wire req_is_write = (state_reg == STATE_WRITE_REQ);

  vc_MemReqMsgToBits #(p_addr_sz, p_data_sz) mem_req_pack (
    .type (req_is_write ? `VC_MEM_REQ_MSG_TYPE_WRITE : `VC_MEM_REQ_MSG_TYPE_READ),
    .addr (cur_word_addr),
    .len  (2'd0),
    .data (cur_word_data),
    .bits (memreq_msg)
  );

  assign memreq_val  = (state_reg == STATE_WRITE_REQ || state_reg == STATE_READ_REQ);
  // Also accept responses during STATE_READ_REQ so rand-delay buffer never fills
  // and backpressure (memreq_rdy=0) cannot deadlock the request-issue loop.
  assign memresp_rdy = (state_reg == STATE_WRITE_RESP ||
                        state_reg == STATE_READ_REQ   ||
                        state_reg == STATE_READ_RESP);
  assign dn_req_rdy  = (state_reg == STATE_IDLE);

  //----------------------------------------------------------------------
  // SWAP response packing
  // Layout (MSB→LSB): has_data | refill_data
  //----------------------------------------------------------------------

  assign dn_resp_msg = {lat_has_refill, line_buf}; // has_data=1 only if we did a refill
  assign dn_resp_val = (state_reg == STATE_DONE);

  //----------------------------------------------------------------------
  // Sequential updates: latch, word counter, line buffer
  //----------------------------------------------------------------------

  always @(posedge clk) begin
    if (reset) begin
      word_cnt       <= {c_word_cnt_sz{1'b0}};
      resp_cnt       <= {c_word_cnt_sz{1'b0}};
      line_buf       <= {c_line_bits{1'b0}};
      lat_has_refill   <= 1'b0;
      lat_has_victim   <= 1'b0;
      lat_victim_dirty <= 1'b0;
      lat_refill_addr  <= {p_addr_sz{1'b0}};
      lat_victim_addr  <= {p_addr_sz{1'b0}};
      lat_victim_data  <= {c_line_bits{1'b0}};
    end else begin

      // Latch SWAP request on entry from IDLE
      if (state_reg == STATE_IDLE && dn_req_val) begin
        lat_has_refill   <= req_has_refill;
        lat_has_victim   <= req_has_victim;
        lat_victim_dirty <= req_victim_dirty;
        lat_refill_addr  <= req_refill_addr;
        lat_victim_addr  <= req_victim_addr;
        lat_victim_data  <= req_victim_data;
        word_cnt         <= {c_word_cnt_sz{1'b0}};
      end

      // Advance word counter on each accepted write request (not at last word)
      if (state_reg == STATE_WRITE_REQ && memreq_rdy && word_cnt != c_words_per_line-1)
        word_cnt <= word_cnt + 1'b1;

      // Reset counter when entering WRITE_RESP (start collecting acks from 0)
      if (state_reg == STATE_WRITE_REQ && state_next == STATE_WRITE_RESP)
        word_cnt <= {c_word_cnt_sz{1'b0}};

      // Advance counter on each received write ack (not at last ack)
      if (state_reg == STATE_WRITE_RESP && memresp_val && word_cnt != c_words_per_line-1)
        word_cnt <= word_cnt + 1'b1;

      // Reset both counters when entering READ_REQ
      if (state_next == STATE_READ_REQ && state_reg != STATE_READ_REQ) begin
        word_cnt <= {c_word_cnt_sz{1'b0}};
        resp_cnt <= {c_word_cnt_sz{1'b0}};
      end

      // READ_REQ: advance request counter (for address generation), not past last word
      if (state_reg == STATE_READ_REQ && memreq_rdy && word_cnt != c_words_per_line-1)
        word_cnt <= word_cnt + 1'b1;

      // Assemble line_buf from read responses in both READ_REQ and READ_RESP.
      // Using resp_cnt (independent of word_cnt) prevents deadlock: memresp_rdy=1
      // during READ_REQ keeps the rand-delay buffer draining so memreq_rdy stays high.
      if ((state_reg == STATE_READ_REQ || state_reg == STATE_READ_RESP) &&
           memresp_val && resp_cnt < c_words_per_line) begin
        line_buf[resp_cnt * p_data_sz +: p_data_sz] <= memresp_data;
        resp_cnt <= resp_cnt + 1'b1;
      end

      // Reset counters on DONE → IDLE
      if (state_reg == STATE_DONE && dn_resp_rdy) begin
        word_cnt <= {c_word_cnt_sz{1'b0}};
        resp_cnt <= {c_word_cnt_sz{1'b0}};
      end

    end
  end

endmodule

`endif /* CACHE_L3_MEM_ADAPTER_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
