//========================================================================
// riscv-custom2.S
//========================================================================

#include "riscv-macros.h"

    TEST_RISCV_BEGIN

    // WAW should succeed for both I2O2 and I2OI 
    // since 'bne' will stall until result of 'mul' becomes ready 
    // since the mul is tracked in the scoreboard

    li    x2, 2;                                                    \
    li    x3, 8;
    add   x4, x2, x3;
    mul   x4, x2, x3;
    
    TEST_CHECK_EQ( x4, 16 );  

    // WAW should succeed for both I2O2 and I2OI 
    // since 'mul' will have enough time to commit before 'add' commits  

    li    x2, 2;                                                    \
    li    x3, 8;
    mul   x4, x2, x3;
    nop; nop; nop; 
    add   x4, x2, x3;

    TEST_CHECK_EQ( x4, 10 );  
    
    TEST_RISCV_END