//=========================================================================
// L3 Cache ↔ Main Memory Adapter  (Inclusive INCL → word-sized VC_MEM)
//=========================================================================
// Bridges L3's line-sized INCL downstream interface to the word-sized
// VC_MEM interface of vc_TestDualPortRandDelayMem.
//
// A single INCL request is decomposed into word-sized memory transactions:
//
//   has_victim=0  →  c_words_per_line READ  requests
//   has_victim=1  →  c_words_per_line WRITEs (victim always dirty in inclusive)
//                    then c_words_per_line READs  (refill always needed)
//
// Every request always generates a refill (no has_refill bit).
// Every sent victim is always dirty (no victim_dirty bit).

`ifndef CACHE_L3_MEM_ADAPTER_V
`define CACHE_L3_MEM_ADAPTER_V

`include "vc-MemReqMsg.v"
`include "vc-MemRespMsg.v"
`include "cache-InclMsg.vh"

module cache_L3MemAdapter
#(
  parameter p_line_sz  = 16,
  parameter p_addr_sz  = 32,
  parameter p_data_sz  = 32
)(
  input clk,
  input reset,

  input  [`CACHE_INCL_REQ_SZ(p_line_sz)-1:0]   dn_req_msg,
  input                                          dn_req_val,
  output                                         dn_req_rdy,

  output [`CACHE_INCL_RESP_SZ(p_line_sz)-1:0]  dn_resp_msg,
  output                                         dn_resp_val,
  input                                          dn_resp_rdy,

  output [`VC_MEM_REQ_MSG_SZ(p_addr_sz,p_data_sz)-1:0]  memreq_msg,
  output                                                  memreq_val,
  input                                                   memreq_rdy,

  input  [`VC_MEM_RESP_MSG_SZ(p_data_sz)-1:0]            memresp_msg,
  input                                                   memresp_val,
  output                                                  memresp_rdy
);

  localparam c_line_bits      = p_line_sz * 8;
  localparam c_bytes_per_word = p_data_sz / 8;
  localparam c_words_per_line = p_line_sz / c_bytes_per_word;
  localparam c_word_cnt_sz    = $clog2(c_words_per_line + 1);

  //----------------------------------------------------------------------
  // Unpack incoming INCL request
  // Layout: has_victim | victim_addr | refill_addr | victim_data
  //----------------------------------------------------------------------

  wire [c_line_bits-1:0]  req_victim_data = dn_req_msg[c_line_bits-1    : 0];
  wire [p_addr_sz-1:0]    req_refill_addr = dn_req_msg[c_line_bits+31   : c_line_bits];
  wire [p_addr_sz-1:0]    req_victim_addr = dn_req_msg[c_line_bits+63   : c_line_bits+32];
  wire                     req_has_victim  = dn_req_msg[c_line_bits+64];

  wire [p_data_sz-1:0] memresp_data = memresp_msg[p_data_sz-1:0];

  //----------------------------------------------------------------------
  // FSM
  //----------------------------------------------------------------------

  localparam STATE_IDLE       = 3'd0;
  localparam STATE_WRITE_REQ  = 3'd1;
  localparam STATE_WRITE_RESP = 3'd2;
  localparam STATE_READ_REQ   = 3'd3;
  localparam STATE_READ_RESP  = 3'd4;
  localparam STATE_DONE       = 3'd5;

  reg [2:0] state_reg, state_next;

  reg [c_word_cnt_sz-1:0] word_cnt;
  reg [c_word_cnt_sz-1:0] resp_cnt;
  reg [c_line_bits-1:0]   line_buf;

  reg                   lat_has_victim;
  reg [p_addr_sz-1:0]   lat_refill_addr;
  reg [p_addr_sz-1:0]   lat_victim_addr;
  reg [c_line_bits-1:0] lat_victim_data;

  always @(posedge clk) begin
    if (reset) state_reg <= STATE_IDLE;
    else       state_reg <= state_next;
  end

  always @(*) begin
    state_next = state_reg;
    case (state_reg)

      STATE_IDLE: begin
        if (dn_req_val) begin
          // Victim is always dirty in inclusive; write if present, always read.
          if (req_has_victim)
            state_next = STATE_WRITE_REQ;
          else
            state_next = STATE_READ_REQ;
        end
      end

      STATE_WRITE_REQ: begin
        if (memreq_rdy && word_cnt == c_words_per_line-1)
          state_next = STATE_WRITE_RESP;
      end

      STATE_WRITE_RESP: begin
        if (resp_cnt >= c_words_per_line ||
            (resp_cnt == c_words_per_line-1 && memresp_val))
          state_next = STATE_READ_REQ;
      end

      STATE_READ_REQ: begin
        if (memreq_rdy && word_cnt == c_words_per_line-1)
          state_next = STATE_READ_RESP;
      end

      STATE_READ_RESP: begin
        if (resp_cnt >= c_words_per_line ||
            (resp_cnt == c_words_per_line-1 && memresp_val))
          state_next = STATE_DONE;
      end

      STATE_DONE: begin
        if (dn_resp_rdy)
          state_next = STATE_IDLE;
      end

      default: state_next = STATE_IDLE;

    endcase
  end

  wire [p_addr_sz-1:0] cur_base_addr =
    (state_reg == STATE_WRITE_REQ || state_reg == STATE_WRITE_RESP)
      ? lat_victim_addr : lat_refill_addr;

  wire [p_addr_sz-1:0] cur_word_addr =
    cur_base_addr + {{(p_addr_sz-c_word_cnt_sz-2){1'b0}}, word_cnt, 2'b00};

  wire [p_data_sz-1:0] cur_word_data = lat_victim_data[word_cnt * p_data_sz +: p_data_sz];

  wire req_is_write = (state_reg == STATE_WRITE_REQ);

  vc_MemReqMsgToBits #(p_addr_sz, p_data_sz) mem_req_pack (
    .type (req_is_write ? `VC_MEM_REQ_MSG_TYPE_WRITE : `VC_MEM_REQ_MSG_TYPE_READ),
    .addr (cur_word_addr),
    .len  (2'd0),
    .data (cur_word_data),
    .bits (memreq_msg)
  );

  assign memreq_val  = (state_reg == STATE_WRITE_REQ || state_reg == STATE_READ_REQ);
  assign memresp_rdy = (state_reg == STATE_WRITE_REQ  ||
                        state_reg == STATE_WRITE_RESP  ||
                        state_reg == STATE_READ_REQ    ||
                        state_reg == STATE_READ_RESP);
  assign dn_req_rdy  = (state_reg == STATE_IDLE);

  // Response always has data (all requests need refill in inclusive)
  assign dn_resp_msg = {1'b1, line_buf};
  assign dn_resp_val = (state_reg == STATE_DONE);

  always @(posedge clk) begin
    if (reset) begin
      word_cnt        <= {c_word_cnt_sz{1'b0}};
      resp_cnt        <= {c_word_cnt_sz{1'b0}};
      line_buf        <= {c_line_bits{1'b0}};
      lat_has_victim  <= 1'b0;
      lat_refill_addr <= {p_addr_sz{1'b0}};
      lat_victim_addr <= {p_addr_sz{1'b0}};
      lat_victim_data <= {c_line_bits{1'b0}};
    end else begin

      if (state_reg == STATE_IDLE && dn_req_val) begin
        lat_has_victim  <= req_has_victim;
        lat_refill_addr <= req_refill_addr;
        lat_victim_addr <= req_victim_addr;
        lat_victim_data <= req_victim_data;
        word_cnt        <= {c_word_cnt_sz{1'b0}};
        resp_cnt        <= {c_word_cnt_sz{1'b0}};
      end

      if (state_reg == STATE_WRITE_REQ && memreq_rdy && word_cnt != c_words_per_line-1)
        word_cnt <= word_cnt + 1'b1;

      if (state_reg == STATE_READ_REQ && memreq_rdy && word_cnt != c_words_per_line-1)
        word_cnt <= word_cnt + 1'b1;

      if ((state_reg == STATE_WRITE_REQ || state_reg == STATE_WRITE_RESP) &&
           memresp_val && resp_cnt < c_words_per_line)
        resp_cnt <= resp_cnt + 1'b1;

      if ((state_reg == STATE_READ_REQ || state_reg == STATE_READ_RESP) &&
           memresp_val && resp_cnt < c_words_per_line) begin
        line_buf[resp_cnt * p_data_sz +: p_data_sz] <= memresp_data;
        resp_cnt <= resp_cnt + 1'b1;
      end

      if (state_next == STATE_READ_REQ && state_reg != STATE_READ_REQ) begin
        word_cnt <= {c_word_cnt_sz{1'b0}};
        resp_cnt <= {c_word_cnt_sz{1'b0}};
      end

      if (state_reg == STATE_DONE && dn_resp_rdy) begin
        word_cnt <= {c_word_cnt_sz{1'b0}};
        resp_cnt <= {c_word_cnt_sz{1'b0}};
      end

    end
  end

endmodule

`endif /* CACHE_L3_MEM_ADAPTER_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
