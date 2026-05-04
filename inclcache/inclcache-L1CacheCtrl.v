// L1 cache controller (inclusive hierarchy).
// On miss: has_victim=refill_evict_valid&&dirty; clean evictions not sent (L2 already has copy).

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

  input                   up_req_val,
  output reg              up_req_rdy,

  input  [p_addr_sz-1:0]  req_addr_lat,
  input                   req_type_lat,

  output reg              up_resp_val,

  output reg              dn_req_val,
  input                   dn_req_rdy,

  output reg              dn_req_has_victim,
  output reg [p_addr_sz-1:0] dn_req_refill_addr,

  input                   dn_resp_val,
  output reg              dn_resp_rdy,

  output reg              refill_tag_wen_en,
  output reg              refill_data_wen_en,
  output reg              mark_dirty,
  output reg              lru_update_refill_en,
  output reg              store_hit_wen_en,
  output reg              victim_swap_en,

  input                   hit,
  input                   victim_hit,
  input                   refill_evict_valid,
  input                   refill_evict_dirty
);

  localparam STATE_IDLE      = 3'd0;
  localparam STATE_TAG_CHECK = 3'd1;
  localparam STATE_HIT_WAIT  = 3'd2;
  localparam STATE_MISS_REQ  = 3'd3;
  localparam STATE_MISS_WAIT = 3'd4;
  localparam STATE_REFILL_WR = 3'd5;
  localparam STATE_RESP      = 3'd6;
  localparam STATE_VICTIM_SWAP = 3'd7;

  reg [2:0] state_reg, state_next;

  localparam c_hit_cnt_sz = $clog2(p_hit_lat > 1 ? p_hit_lat : 2);
  reg [c_hit_cnt_sz-1:0] hit_lat_cnt;

  localparam c_offset_sz = $clog2(p_line_sz);

  always @(posedge clk) begin
    if (reset) state_reg <= STATE_IDLE;
    else       state_reg <= state_next;
  end

  always @(posedge clk) begin
    if (reset) begin
      hit_lat_cnt <= {c_hit_cnt_sz{1'b0}};
    end else begin
      if (state_reg == STATE_TAG_CHECK && state_next == STATE_HIT_WAIT)
        hit_lat_cnt <= p_hit_lat - 2;
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
        if (hit)
          state_next = (p_hit_lat <= 1) ? STATE_RESP : STATE_HIT_WAIT;
        else if (victim_hit)
          state_next = STATE_VICTIM_SWAP;
        else
          state_next = STATE_MISS_REQ;
      end

      STATE_HIT_WAIT: begin
        if (hit_lat_cnt == {c_hit_cnt_sz{1'b0}})
          state_next = STATE_RESP;
      end

      STATE_MISS_REQ: begin
        if (dn_req_rdy)
          state_next = STATE_MISS_WAIT;
      end

      STATE_MISS_WAIT: begin
        if (dn_resp_val)
          state_next = STATE_REFILL_WR;
      end

      STATE_REFILL_WR: begin
        state_next = STATE_RESP;
      end

      STATE_VICTIM_SWAP: begin
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
    dn_req_refill_addr   = {p_addr_sz{1'b0}};
    dn_resp_rdy          = 1'b0;
    refill_tag_wen_en    = 1'b0;
    refill_data_wen_en   = 1'b0;
    mark_dirty           = 1'b0;
    lru_update_refill_en = 1'b0;
    store_hit_wen_en     = 1'b0;
    victim_swap_en       = 1'b0;

    case (state_reg)

      STATE_IDLE: begin
        up_req_rdy = 1'b1;
      end

      STATE_TAG_CHECK: begin
        if (hit && req_type_lat)
          store_hit_wen_en = 1'b1;
      end

      STATE_HIT_WAIT: begin
      end

      STATE_MISS_REQ: begin
        dn_req_val        = 1'b1;
        // Only include victim if it was occupied AND dirty — clean lines
        // are already correct in L2 (inclusive invariant).
        dn_req_has_victim = refill_evict_valid && refill_evict_dirty;
        dn_req_refill_addr = {req_addr_lat[p_addr_sz-1:c_offset_sz], {c_offset_sz{1'b0}}};
      end

      STATE_MISS_WAIT: begin
        dn_resp_rdy = 1'b1;
      end

      STATE_REFILL_WR: begin
        refill_tag_wen_en    = 1'b1;
        refill_data_wen_en   = 1'b1;
        mark_dirty           = req_type_lat;
        lru_update_refill_en = 1'b1;
      end

      STATE_VICTIM_SWAP: begin
        refill_tag_wen_en    = 1'b1;
        refill_data_wen_en   = 1'b1;
        mark_dirty           = req_type_lat;
        lru_update_refill_en = 1'b1;
        victim_swap_en       = 1'b1;
      end

      STATE_RESP: begin
        up_resp_val          = 1'b1;
        lru_update_refill_en = 1'b1;
      end

      default: begin
        up_req_rdy = 1'b0;
      end

    endcase
  end

endmodule

`endif /* CACHE_L1_CACHE_CTRL_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
