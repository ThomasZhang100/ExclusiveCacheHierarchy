//========================================================================
// riscv-custom.S
//========================================================================

#include "riscv-macros.h"

    TEST_RISCV_BEGIN

    // WAW should fail for I2O2 — mul commits after add 

    li    x2, 2;                                                    \
    li    x3, 8;
    mul   x4, x2, x3;
    add   x4, x2, x3;
    nop; nop; nop; nop; 

    TEST_CHECK_EQ( x4, 10 );  
    
    TEST_RISCV_END