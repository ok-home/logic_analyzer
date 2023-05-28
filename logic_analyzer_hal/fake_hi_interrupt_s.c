//
// fake interrupt handler
// for create asm file
// /home/ok-home/.espressif/tools/xtensa-esp32-elf/esp-2022r1-11.2.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc /home/ok-home/myex/logic_analizer/logic_analyzer_hal/fake_hi_interrupt_s.c -S -O3
// header
/*

#include <xtensa/coreasm.h>
#include <xtensa/corebits.h>
#include <xtensa/config/system.h>
#include <xtensa/hal.h>


.data
_l5_intr_stack:
.space      60

    .section .iram1,"ax" 
    .global     xt_highint5   
    .type       xt_highint5,@function
    .align      4
xt_highint5:

    movi    a0,     _l5_intr_stack
    s32i    a3,     a0,     0    
    s32i    a4,     a0,     4   
*/
// footer
/*
SAVE_EXIT:
    movi    a0,     _l5_intr_stack
    l32i    a3,     a0,     0
    l32i    a4,     a0,     4

    rsr     a0, EXCSAVE_5   
    rfi     5

.global ld_include_my_isr_file
ld_include_my_isr_file:
*/
//
#include <stdint.h>
#define _REG_READ(_r)        (*(volatile uint32_t *)(_r))
#define _REG_WRITE(_r, _v)   (*(volatile uint32_t *)(_r)) = (_v)

#include "./include/logic_analyzer_hi_lewel_interrupt.h"

extern hi_interrupt_state_t la_hi_interrupt_state;

void xt_highint5(void)
{
    // save reg
    //  default use int31
    //  start dma - set vsync bit to 1
    _REG_WRITE(la_hi_interrupt_state.i2s_set_vsync_reg, la_hi_interrupt_state.i2s_set_vsync_bit);
    // disable interrupt on core
    _REG_WRITE(la_hi_interrupt_state.dport_int_map_reg, la_hi_interrupt_state.dport_int_map_data_disable);
    // clear GPIO interrupt enable on core
    _REG_WRITE(la_hi_interrupt_state.gpio_pin_cfg_reg, la_hi_interrupt_state.gpio_pin_cfg_backup_data);
    // clear interrupt status if not shared
    //_REG_WRITE(la_hi_interrupt_state.gpio_stat_clr_reg, la_hi_interrupt_state.gpio_mask);
    // restore reg
    // iret
}
