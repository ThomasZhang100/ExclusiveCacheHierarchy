//========================================================================
// riscv-custom1.S
//========================================================================

#include "riscv-macros.h"

    TEST_RISCV_BEGIN

    // WAW should succeed for both I2O2 (under non-random memory delay) and I2OI 
    // since we check value of r4 before the 'mul' commits but after 'add' commits

    li    x2, 2;                                                    \
    li    x3, 8;
    mul   x4, x2, x3;
    add   x4, x2, x3;

    TEST_CHECK_EQ( x4, 10 );  
    
    TEST_RISCV_END