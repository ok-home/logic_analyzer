#include <xtensa/coreasm.h>
#include <xtensa/corebits.h>
#include <xtensa/config/system.h>
#include <xtensa/hal.h>
#include <xtensa/config/specreg.h>


.data
_l5_intr_stack:
.space      60

    .section .iram1,"ax" 
	.literal_position
	.literal .LC0, la_hi_interrupt_state
	.align	4
	.global	xt_highint5
	.type	xt_highint5, @function

xt_highint5:
//
    movi    a0,     _l5_intr_stack
    s32i    a8,     a0,     0    
    s32i    a9,     a0,     4 
    s32i    a10,    a0,     8    
//
	l32r	a8, .LC0
	l32i.n	a9, a8, 36
	l32i.n	a10, a8, 40
	memw
	s32i.n	a10, a9, 0
	l32i.n	a9, a8, 4
	l32i.n	a10, a8, 8
	memw
	s32i.n	a10, a9, 0
	l32i.n	a9, a8, 24
	l32i.n	a10, a8, 16
	memw
	s32i.n	a10, a9, 0
	l32i.n	a9, a8, 28
	l32i.n	a10, a8, 32
	memw
	l32i.n	a8, a9, 0
	and	a8, a8, a10
	memw
	s32i.n	a8, a9, 0
//
    movi    a0,     _l5_intr_stack
    l32i    a8,     a0,     0
    l32i    a9,     a0,     4
    l32i    a10,    a0,     8
//
	rsr.excsave5 a0
    //rsr     a0, EXCSAVE_5   
	//rsr     a0, 213   
    rfi     5

.global la_include_hi_interrupt
la_include_hi_interrupt:

