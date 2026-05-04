// Inclusive L2/L3 cache controller FSM.
// On miss: sends INCL request downstream with dirty eviction victim if LRU slot occupied and dirty.

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

  input                      up_req_val,
  output reg                 up_req_rdy,

  // Latched INCL request fields (supplied by enclosing BaseCache)
  input                      lat_has_victim,
  input  [p_addr_sz-1:0]     lat_refill_addr,

  output reg                 up_resp_val,
  input                      up_resp_rdy,

  output reg                 dn_req_val,
  input                      dn_req_rdy,

  // Downstream request fields driven by ctrl; victim data/addr from dpath
  output reg                 dn_req_has_victim,
  output reg [p_addr_sz-1:0] dn_req_refill_addr,

  input                      dn_resp_val,
  output reg                 dn_resp_rdy,

  // Dpath write-enable outputs
  output reg                 refill_tag_wen_en,
  output reg                 refill_data_wen_en,
  output reg                 mark_dirty,
  output reg                 lru_update_refill_en,
  output reg                 store_hit_wen_en,

  output reg                 victim_tag_wen_en,
  output reg                 victim_data_wen_en,

  // High exactly during STATE_TAG_CHECK (for hit_line capture in wrapper)
  output reg                 tag_check,

  // Dpath status inputs
  input                      hit,
  input                      refill_evict_valid,
  input                      refill_evict_dirty,

  // req_type_lat: only needed at L1 for store-hit; tied 0 at L2/L3
  input                      req_type_lat
);

  localparam STATE_IDLE         = 4'd0;
  localparam STATE_TAG_CHECK    = 4'd1;
  localparam STATE_WRITE_VICTIM = 4'd2;
  localparam STATE_HIT_WAIT     = 4'd3;
  localparam STATE_DN_REQ       = 4'd4;
  localparam STATE_DN_WAIT      = 4'd5;
  localparam STATE_REFILL_WR    = 4'd6;
  localparam STATE_RESP         = 4'd7;

  reg [3:0] state_reg, state_next;

  localparam c_hit_cnt_sz = $clog2(p_hit_lat > 1 ? p_hit_lat : 2);
  reg [c_hit_cnt_sz-1:0] hit_lat_cnt;

  always @(posedge clk) begin
    if (reset) state_reg <= STATE_IDLE;
    else       state_reg <= state_next;
  end

  always @(posedge clk) begin
    if (reset) begin
      hit_lat_cnt <= {c_hit_cnt_sz{1'b0}};
    end else begin
      if ((state_reg == STATE_TAG_CHECK || state_reg == STATE_WRITE_VICTIM) &&
           state_next == STATE_HIT_WAIT)
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
        if (lat_has_victim)
          state_next = STATE_WRITE_VICTIM;
        else if (hit)
          state_next = (p_hit_lat <= 1) ? STATE_RESP : STATE_HIT_WAIT;
        else
          state_next = STATE_DN_REQ;
      end

      STATE_WRITE_VICTIM: begin
        // Victim write completes this cycle; evaluate refill hit/miss next.
        if (hit)
          state_next = (p_hit_lat <= 1) ? STATE_RESP : STATE_HIT_WAIT;
        else
          state_next = STATE_DN_REQ;
      end

      STATE_HIT_WAIT: begin
        if (hit_lat_cnt == {c_hit_cnt_sz{1'b0}})
          state_next = STATE_RESP;
      end

      STATE_DN_REQ: begin
        if (dn_req_rdy)
          state_next = STATE_DN_WAIT;
      end

      STATE_DN_WAIT: begin
        if (dn_resp_val)
          state_next = STATE_REFILL_WR;
      end

      STATE_REFILL_WR: begin
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
    victim_tag_wen_en    = 1'b0;
    victim_data_wen_en   = 1'b0;
    tag_check            = 1'b0;

    case (state_reg)

      STATE_IDLE: begin
        up_req_rdy = 1'b1;
      end

      STATE_TAG_CHECK: begin
        tag_check = 1'b1;
        if (hit && req_type_lat)
          store_hit_wen_en = 1'b1;
      end

      STATE_WRITE_VICTIM: begin
        // Write incoming victim data into its way in vic_set_idx.
        victim_tag_wen_en  = 1'b1;
        victim_data_wen_en = 1'b1;
      end

      STATE_HIT_WAIT: begin
      end

      STATE_DN_REQ: begin
        dn_req_val        = 1'b1;
        // Include eviction victim only if the LRU slot is occupied and dirty.
        dn_req_has_victim = refill_evict_valid && refill_evict_dirty;
        dn_req_refill_addr = lat_refill_addr;
      end

      STATE_DN_WAIT: begin
        dn_resp_rdy = 1'b1;
      end

      STATE_REFILL_WR: begin
        refill_tag_wen_en  = 1'b1;
        refill_data_wen_en = 1'b1;
        // mark_dirty stays 0 at L2/L3 — stores are write-allocated at L1
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

`endif /* CACHE_BASE_CACHE_CTRL_V */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
