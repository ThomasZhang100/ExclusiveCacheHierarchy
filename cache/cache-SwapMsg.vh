// SWAP message format (W = line_sz*8)
// Request (67+W bits, MSB→LSB): has_victim(1) | has_refill(1) | victim_dirty(1) | victim_addr(32) | refill_addr(32) | victim_data(W)
//   has_victim=0,has_refill=1: cold miss; 1,1: swap; 1,0: pure eviction
// Response (1+W bits, MSB→LSB): has_data(1) | refill_data(W)  [has_data=0: eviction ack]

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
