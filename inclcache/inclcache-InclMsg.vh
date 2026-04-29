//=========================================================================
// Cache Inclusive Message Format
//=========================================================================
//
// -----------------------------------------------------------------------
// INCL Request  (sent downstream: cache → next level)
// -----------------------------------------------------------------------
//
//  Bit layout (MSB → LSB):
//
//   [W+64]       has_victim   (1b)  – 1: request includes a dirty victim to donate
//   [W+63:W+32]  victim_addr  (32b) – cache-line-aligned address of victim
//   [W+31:0]     refill_addr  (32b) – cache-line-aligned address to fetch
//   [W-1:0]      victim_data  (W b) – data of victim line  (W = line_sz*8)
//
//   Total bits: 65 + line_sz*8
//
// has_victim=0 → cold miss (no dirty victim; clean lines need no writeback)
// has_victim=1 → dirty victim donation + refill
//
// There is no has_refill bit: every request always needs the refill data back.
// There is no victim_dirty bit: only dirty victims are sent downstream.
//
// -----------------------------------------------------------------------
// INCL Response  (sent upstream: next level → cache)
// -----------------------------------------------------------------------
//
//  Bit layout (MSB → LSB):
//
//   [W]      has_data    (1b)  – always 1 in the inclusive hierarchy
//   [W-1:0]  refill_data (W b) – the requested cache line
//
//   Total bits: 1 + line_sz*8

`ifndef CACHE_INCL_MSG_VH
`define CACHE_INCL_MSG_VH

// Total message sizes
`define CACHE_INCL_REQ_SZ(line_sz)  (65 + (line_sz)*8)
`define CACHE_INCL_RESP_SZ(line_sz) (1  + (line_sz)*8)

// Request field bit positions (use with a localparam c_line_bits = line_sz*8)
// victim_data  : [c_line_bits-1    : 0]
// refill_addr  : [c_line_bits+31   : c_line_bits]
// victim_addr  : [c_line_bits+63   : c_line_bits+32]
// has_victim   : [c_line_bits+64]

// Response field bit positions
// refill_data  : [c_line_bits-1 : 0]
// has_data     : [c_line_bits]

`endif /* CACHE_INCL_MSG_VH */

// vim: set textwidth=0 ts=2 sw=2 sts=2 :
