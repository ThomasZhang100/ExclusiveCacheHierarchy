// INCL message format (W = line_sz*8)
// Request (65+W bits, MSB->LSB): has_victim(1) | victim_addr(32) | refill_addr(32) | victim_data(W)
//   has_victim=1: dirty victim donation; all requests need refill back; clean evictions not sent
// Response (1+W bits, MSB->LSB): has_data(1, always 1) | refill_data(W)

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
