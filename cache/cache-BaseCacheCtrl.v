//=========================================================================
// L2 / L3 Cache Controller  (SWAP protocol, set-index aware)
//=========================================================================
`ifndef CACHE_BASE_CACHE_CTRL_V
`define CACHE_BASE_CACHE_CTRL_V

module cache_BaseCacheCtrl
#(
  parameter p_num_sets  = 256,
  parameter p_num_ways  = 1,
  parameter p_line_sz   = 16,
  parameter p_addr_sz   = 32,
  parameter p_data_sz   = 32,
  parameter p_hit_lat   = 4
)(
  input clk,
  input reset,

  input                       up_req_val,
  output reg                  up_req_rdy,

  input                       up_req_has_victim,
  input                       up_req_has_refill,
  input  [p_addr_sz-1:0]      up_req_victim_addr,
  input  [p_addr_sz-1:0]      up_req_refill_addr,

  output reg                  up_resp_val,
  input                       up_resp_rdy,

  output reg                  dn_req_val,
  input                       dn_req_rdy,

  output reg                  dn_req_has_victim,
  output reg                  dn_req_has_refill,
  output reg [p_addr_sz-1:0]  dn_req_victim_addr,
  output reg [p_addr_sz-1:0]  dn_req_refill_addr,

  input                       dn_resp_val,
  output reg                  dn_resp_rdy,
  input                       dn_resp_has_data,

  output reg                  refill_tag_wen_en,
  output reg                  refill_data_wen_en,
  output reg                  refill_invalidate_en,

  output reg                  victim_tag_wen_en,
  output reg                  victim_data_wen_en,
  output reg                  victim_set_evict_en,

  output reg                  mark_dirty,
  output reg                  lru_update_refill_en,
  output reg                  lru_update_victim_en,

  // tells enclosing module to use incoming_victim_data (not dn_refill_data) for refill_line
  output reg                  inplace_swap,

  // high exactly during STATE_TAG_CHECK (for hit_line capture in enclosing module)
  output reg                  tag_check,

  input                       hit,
  input                       same_set,

  input                       refill_evict_valid,
  input                       refill_evict_dirty,
  input  [p_addr_sz-1:0]      refill_evict_addr,

  input                       victim_set_has_free,
  input                       victim_set_evict_dirty
);

  localparam STATE_IDLE          = 4'd0;
  localparam STATE_TAG_CHECK     = 4'd1;
  localparam STATE_INPLACE_SWAP  = 4'd2;
  localparam STATE_HIT_WAIT      = 4'd3;
  localparam STATE_EVICT_V_REQ   = 4'd4;
  localparam STATE_EVICT_V_WAIT  = 4'd5;
  localparam STATE_PLACE_V       = 4'd6;
  localparam STATE_INVALIDATE_A  = 4'd7;
  localparam STATE_SWAP_REQ      = 4'd8;
  localparam STATE_SWAP_WAIT     = 4'd9;
  localparam STATE_RESP          = 4'd10;

  reg [3:0] state_reg, state_next;

  localparam c_hit_cnt_sz = $clog2(p_hit_lat > 1 ? p_hit_lat : 2);
  reg [c_hit_cnt_sz-1:0] hit_lat_cnt;

  reg                  lat_has_victim;
  reg                  lat_has_refill;
  reg [p_addr_sz-1:0]  lat_victim_addr;
  reg [p_addr_sz-1:0]  lat_refill_addr;
  // Set on a miss when victim_set was full: V3 (victim_set_evict_*) is piggybacked
  // onto the downstream SWAP as its victim instead of a separate EVICT_V_REQ.
  reg                  lat_v3_displaced;

  always @(posedge clk) begin
    if (reset) begin
      lat_has_victim   <= 1'b0;
      lat_has_refill   <= 1'b0;
      lat_victim_addr  <= {p_addr_sz{1'b0}};
      lat_refill_addr  <= {p_addr_sz{1'b0}};
      lat_v3_displaced <= 1'b0;
    end else begin
      if (up_req_val && up_req_rdy) begin
        lat_has_victim  <= up_req_has_victim;
        lat_has_refill  <= up_req_has_refill;
        lat_victim_addr <= up_req_victim_addr;
        lat_refill_addr <= up_req_refill_addr;
      end
      // Latch whether V3 needs to be evicted from victim_set on a miss.
      // victim_set_has_free is combinational from the dpath; capture it at TAG_CHECK.
      if (state_reg == STATE_TAG_CHECK && !hit)
        lat_v3_displaced <= lat_has_victim && !victim_set_has_free;
    end
  end

  always @(posedge clk) begin
    if (reset) state_reg <= STATE_IDLE;
    else       state_reg <= state_next;
  end

  // Hit-latency counter update
  always @(posedge clk) begin
    if (reset) begin
      hit_lat_cnt <= {c_hit_cnt_sz{1'b0}};
    end else begin
      if (state_reg == STATE_INVALIDATE_A && state_next == STATE_HIT_WAIT)
        hit_lat_cnt <= p_hit_lat - 2; // spend one cycle in INVALIDATE_A already
      else if (state_reg == STATE_HIT_WAIT && hit_lat_cnt != {c_hit_cnt_sz{1'b0}})
        hit_lat_cnt <= hit_lat_cnt - 1'b1;
    end
  end

  always @(*) begin
    state_next = state_reg;
    case (state_reg)

      STATE_IDLE: begin
        if (up_req_val)
          state_next = STATE_TAG_CHECK;
      end

      STATE_TAG_CHECK: begin
        if (hit) begin
          if (lat_has_victim && same_set) // if the victim and the requested address index to the same set, we can swap
            state_next = STATE_INPLACE_SWAP;
          else if (lat_has_victim && !victim_set_has_free)
            state_next = STATE_EVICT_V_REQ;
          else if (lat_has_victim)
            state_next = STATE_PLACE_V;
          else
            state_next = STATE_INVALIDATE_A;
        end else begin
          // Miss: always go directly to SWAP_REQ.
          // If victim_set was full, V3 is piggybacked as the victim in that SWAP
          // (lat_v3_displaced is latched at this edge). This saves a full round
          // trip versus the hit-path's separate EVICT_V_REQ + SWAP_REQ sequence.
          state_next = STATE_SWAP_REQ;
        end
      end

      STATE_INPLACE_SWAP: begin
        state_next = STATE_RESP;
      end

      STATE_HIT_WAIT: begin
        if (hit_lat_cnt == {c_hit_cnt_sz{1'b0}})
          state_next = STATE_RESP;
      end

      STATE_EVICT_V_REQ: begin
        if (dn_req_rdy)
          state_next = STATE_EVICT_V_WAIT;
      end

      STATE_EVICT_V_WAIT: begin
        if (dn_resp_val)
          state_next = STATE_PLACE_V;
      end

      STATE_PLACE_V: begin
        if (hit)
          state_next = STATE_INVALIDATE_A; // hit path: V1 placed, now invalidate A
        else
          state_next = STATE_SWAP_WAIT;    // miss path: SWAP already sent, now wait
      end

      STATE_INVALIDATE_A: begin
        state_next = (p_hit_lat <= 1) ? STATE_RESP : STATE_HIT_WAIT;
      end

      STATE_SWAP_REQ: begin
        if (dn_req_rdy)
          // If there's an incoming victim (V1) to absorb, do it now that
          // V3's slot is being cleared (victim_set_evict_en was asserted this cycle).
          state_next = lat_has_victim ? STATE_PLACE_V : STATE_SWAP_WAIT;
      end

      STATE_SWAP_WAIT: begin
        if (dn_resp_val)
          state_next = STATE_RESP;
      end

      STATE_RESP: begin
        state_next = STATE_IDLE;
      end

      default: state_next = STATE_IDLE;

    endcase
  end

  always @(*) begin
    up_req_rdy           = 1'b0;
    up_resp_val          = 1'b0;
    dn_req_val           = 1'b0;
    dn_req_has_victim    = 1'b0;
    dn_req_has_refill    = 1'b0;
    dn_req_victim_addr   = {p_addr_sz{1'b0}};
    dn_req_refill_addr   = {p_addr_sz{1'b0}};
    dn_resp_rdy          = 1'b0;
    refill_tag_wen_en    = 1'b0;
    refill_data_wen_en   = 1'b0;
    refill_invalidate_en = 1'b0;
    victim_tag_wen_en    = 1'b0;
    victim_data_wen_en   = 1'b0;
    victim_set_evict_en  = 1'b0;
    mark_dirty           = 1'b0;
    lru_update_refill_en = 1'b0;
    lru_update_victim_en = 1'b0;
    inplace_swap         = 1'b0;
    tag_check            = 1'b0;

    case (state_reg)

      STATE_IDLE: begin
        up_req_rdy = 1'b1;
      end

      STATE_TAG_CHECK: begin
        tag_check = 1'b1;
      end

      STATE_INPLACE_SWAP: begin
        // Write incoming victim into A's exact slot (same set, same way as hit_way).
        // The enclosing module must route incoming_victim_data as refill_line
        // and use hit_way as the one-hot write enable.
        inplace_swap         = 1'b1;
        refill_tag_wen_en    = 1'b1;
        refill_data_wen_en   = 1'b1;
        lru_update_refill_en = 1'b1;
      end

      STATE_EVICT_V_REQ: begin
        dn_req_val          = 1'b1;
        dn_req_has_victim   = 1'b1;
        dn_req_has_refill   = 1'b0;
        // victim_addr = victim_set_evict_addr (from dpath, wired in enclosing module)
        victim_set_evict_en = 1'b1;
      end

      STATE_EVICT_V_WAIT: begin
        dn_resp_rdy = 1'b1;
      end

      STATE_PLACE_V: begin
        victim_tag_wen_en    = 1'b1;
        victim_data_wen_en   = 1'b1;
        lru_update_victim_en = 1'b1;
      end

      STATE_INVALIDATE_A: begin
        refill_invalidate_en = 1'b1;
      end

      STATE_SWAP_REQ: begin
        dn_req_val         = 1'b1;
        // Victim is V3 from victim_set when victim_set was full on a miss;
        // otherwise no victim (incoming V1 fit in a free slot, or no incoming victim).
        // L2/L3 never evict from the refill set — they don't keep A.
        dn_req_has_victim  = lat_v3_displaced;
        dn_req_has_refill  = 1'b1;
        dn_req_refill_addr = lat_refill_addr;
        // victim_addr/data/dirty driven from victim_set_evict_* in enclosing module
        victim_set_evict_en = lat_v3_displaced; // clear V3's slot so PLACE_V can fill it
      end

      STATE_SWAP_WAIT: begin
        dn_resp_rdy = 1'b1;
      end

      STATE_RESP: begin
        up_resp_val = 1'b1;
      end

      default: begin
        up_req_rdy = 1'b0;
      end

    endcase
  end

endmodule

`endif /* CACHE_BASE_CACHE_CTRL_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
