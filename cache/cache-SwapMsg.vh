//=========================================================================
// Cache SWAP Message Format
//=========================================================================
//
// -----------------------------------------------------------------------
// SWAP Request  (sent downstream: cache → next level)
// -----------------------------------------------------------------------
//
//  Bit layout (MSB → LSB):
//
//   [W+66]       has_victim   (1b)  – 1: request includes a victim to donate
//   [W+65]       has_refill   (1b)  – 1: request needs a refill line back
//                                        0: pure eviction (no refill needed)
//   [W+64]       victim_dirty (1b)  – 1: victim has been modified (write-back needed
//                                        when it eventually leaves the hierarchy)
//   [W+63:W+32]  victim_addr  (32b) – cache-line-aligned address of victim
//   [W+31:0]     refill_addr  (32b) – cache-line-aligned address to fetch
//   [W-1:0]      victim_data  (W b) – data of victim line  (W = line_sz*8)
//
//   Total bits: 3 + 32 + 32 + line_sz*8 = 67 + line_sz*8
//
// has_victim=0, has_refill=1 → cold miss (no valid victim yet)
// has_victim=1, has_refill=1 → swap (evict victim, fetch refill)
// has_victim=1, has_refill=0 → pure eviction (victim_set overflow)
// has_victim=0, has_refill=0 → invalid / unused
//
// victim_dirty is only meaningful when has_victim=1.
// A clean victim (victim_dirty=0) does not need to be written to main memory
// when it is eventually evicted from L3 — the copy in memory is already correct.
//
// -----------------------------------------------------------------------
// SWAP Response  (sent upstream: next level → cache)
// -----------------------------------------------------------------------
//
//  Bit layout (MSB → LSB):
//
//   [W]      has_data    (1b)  – 1: response contains refill data
//   [W-1:0]  refill_data (W b) – the requested cache line
//
//   Total bits: 1 + line_sz*8
//
//   has_data=0 → pure eviction acknowledgement
//   has_data=1 → refill data valid

`ifndef CACHE_SWAP_MSG_VH
`define CACHE_SWAP_MSG_VH

// Total message sizes
`define CACHE_SWAP_REQ_SZ(line_sz)  (67 + (line_sz)*8)
`define CACHE_SWAP_RESP_SZ(line_sz) (1  + (line_sz)*8)

// Request field bit positions (use with a localparam c_line_bits = line_sz*8)
// victim_data  : [c_line_bits-1    : 0]
// refill_addr  : [c_line_bits+31   : c_line_bits]
// victim_addr  : [c_line_bits+63   : c_line_bits+32]
// victim_dirty : [c_line_bits+64]
// has_refill   : [c_line_bits+65]
// has_victim   : [c_line_bits+66]

// Response field bit positions
// refill_data  : [c_line_bits-1 : 0]
// has_data     : [c_line_bits]

`endif /* CACHE_SWAP_MSG_VH */
