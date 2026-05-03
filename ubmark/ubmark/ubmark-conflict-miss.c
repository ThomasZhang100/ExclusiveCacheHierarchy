//========================================================================
// ubmark-conflict-miss
//========================================================================

#include "ubmark.h"

#define STRIDE_WORDS 1024
#define ITERS 2048

static volatile int conflict_buf[STRIDE_WORDS * 2] __attribute__((aligned(4096)));

//------------------------------------------------------------------------
// conflict_miss_scalar
//------------------------------------------------------------------------

__attribute__ ((noinline))
int conflict_miss_scalar( volatile int* buf )
{
  int sum = 0;
  int i;
  for ( i = 0; i < ITERS; i++ ) {
    sum += buf[0];
    sum += buf[STRIDE_WORDS];
  }
  return sum;
}

//------------------------------------------------------------------------
// Test harness
//------------------------------------------------------------------------

int main( int argc, char* argv[] )
{
  int i;
  for ( i = 0; i < STRIDE_WORDS * 2; i++ )
    conflict_buf[i] = 0;

  conflict_buf[0] = 3;
  conflict_buf[STRIDE_WORDS] = 7;

  int temp = 0;
  int result = 0;

  test_stats_on( temp );
  result = conflict_miss_scalar( conflict_buf );
  test_stats_off( temp );

  if ( result != ITERS * 10 )
    test_fail( result );

  test_pass( temp );
  return 0;
}
