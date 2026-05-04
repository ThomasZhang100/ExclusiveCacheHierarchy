//========================================================================
// ubmark-reuse-scan
//========================================================================
// Repeatedly scans a buffer that is 1.5× the L1 capacity (96 lines =
// 384 ints = 1536 B with default 64-set / 16-byte-line L1).
//
// Access pattern (sequential, many iterations):
//   Lines 0-63  → L1 sets 0-63   (fill L1 exactly)
//   Lines 64-95 → L1 sets 0-31   (conflict-evict lines 0-31 from L1)
//
// Exclusive steady state: lines 0-31 park in L2 after eviction (SWAP),
//   so every pass sees only L1 or L2 hits — no L3 traffic.
//
// Inclusive steady state: L2 mirrors L1 (same geometry, same set index),
//   so lines 0-31 are absent from L2 and must be re-fetched from L3
//   every pass — 64 L3 misses per pass vs. 64 L2 hits.

#include "ubmark.h"

// 96 lines × 4 words/line = 384 ints = 1536 B
// (lines 64-95 alias sets 0-31 of a 64-set L1 and L2, driving the gap)
#define BUF_LINES  96
#define LINE_WORDS 4
#define BUF_SIZE   (BUF_LINES * LINE_WORDS)

// 4096-byte alignment ensures line 0 → set 0, line 1 → set 1, etc.
static volatile int scan_buf[BUF_SIZE] __attribute__((aligned(4096)));

__attribute__((noinline))
int reuse_scan( volatile int* buf, int iters )
{
  int sum = 0;
  int i, iter;
  for ( iter = 0; iter < iters; iter++ ) {
    for ( i = 0; i < BUF_SIZE; i++ ) {
      sum += buf[i];
    }
  }
  return sum;
}

int main( int argc, char* argv[] )
{
  int iters = 50;
  int i;
  for ( i = 0; i < BUF_SIZE; i++ )
    scan_buf[i] = 1;

  int temp = 0;
  int result = 0;

  test_stats_on( temp );
  result = reuse_scan( scan_buf, iters );
  test_stats_off( temp );

  if ( result != iters * BUF_SIZE )
    test_fail( result );

  test_pass( temp );
  return 0;
}
