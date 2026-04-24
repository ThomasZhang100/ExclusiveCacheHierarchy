//========================================================================
// riscv-custom3.S
//========================================================================

#include "riscv-macros.h"

    TEST_RISCV_BEGIN

    // I2OI design must bypass out of ROB — 'add' writes back to ROB the
    // cycle before 'addi' reads; 'mul' hasn't committed yet so neither has
    // the 'add'. 

    li    x2, 2;                                                    \
    li    x3, 8;
    mul   x5, x2, x3;
    add   x4, x2, x3;
    nop; nop; 
    addi  x6, x4, 1; 
    
    TEST_CHECK_EQ( x6, 11 );  
    
    TEST_RISCV_END