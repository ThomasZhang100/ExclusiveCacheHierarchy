//========================================================================
// riscv-custom4.S
//========================================================================

#include "riscv-macros.h"

    TEST_RISCV_BEGIN

    // I4 has better IPC than I2OI here since I2OI has extra 
    // commit stage from the ROB, and mul pipe has 4 stages
    // in both designs. 

    li    x2, 2;                                                    \
    li    x3, 8;
    mul   x5, x2, x3;
    
    TEST_CHECK_EQ( x5, 16 );  
    
    TEST_RISCV_END