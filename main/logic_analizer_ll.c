// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
//#include <string.h>
#include "soc/i2s_struct.h"
#include "soc/gpio_struct.h"
//#include "esp_idf_version.h"
#include "hal/gpio_ll.h"
#include <driver/gpio.h>
#include "esp_rom_gpio.h"

#include "logic_analizer_hal.h"
#include "logic_analizer_ll.h"

#define LA_CLK_SAMPLE_RATE 80000000
//#define GPIO_PIN_INTR_POSEDGE GPIO_INTR_POSEDGE
//#define GPIO_PIN_INTR_NEGEDGE GPIO_INTR_NEGEDGE

#define gpio_matrix_in(a, b, c) esp_rom_gpio_connect_in_signal(a, b, c)

#define I2S_ISR_ENABLE(i)   \
    {                       \
        I2S0.int_clr.i = 1; \
        I2S0.int_ena.i = 1; \
    }
#define I2S_ISR_DISABLE(i)  \
    {                       \
        I2S0.int_ena.i = 0; \
        I2S0.int_clr.i = 1; \
    }

typedef struct div_68
{
    uint8_t div_8;
    uint8_t div_6;
} div_68_t;

typedef enum
{
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, ...
     */
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
    SM_0A00_0B00 = 3,
} i2s_sampling_mode_t;

static intr_handle_t isr_handle;

static void IRAM_ATTR la_ll_dma_isr(void *handle)
{
    BaseType_t HPTaskAwoken = pdFALSE;

    typeof(I2S0.int_st) status = I2S0.int_st;
    if (status.val == 0)
    {
        return;
    }
    I2S0.int_clr.val = status.val;

    if (status.in_suc_eof)
    {

        vTaskNotifyGiveFromISR((TaskHandle_t)handle, &HPTaskAwoken);
    }
    if (HPTaskAwoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}
static void logic_analizer_ll_reset()
{
    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
}
static  void logic_analizer_ll_set_mode()
{
    /*
    I2S0.conf.rx_slave_mod = 0; //- master mode
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    */
    I2S0.conf.val = 0;

    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;

    I2S0.fifo_conf.dscr_en = 1;
    I2S0.fifo_conf.rx_fifo_mod = SM_0A0B_0C0D;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;

    I2S0.conf_chan.rx_chan_mod = 1;
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.timing.val = 0;
    //I2S0.timing.rx_dsync_sw = 0; //1
}
static div_68_t logic_analizer_ll_convert_sample_rate(int sample_rate)
{
        div_68_t ldiv;
       int cnt_div = LA_CLK_SAMPLE_RATE/sample_rate;
       if( cnt_div < 64 ) 
       {
            ldiv.div_8=2; 
            ldiv.div_6=cnt_div/2;
        }
        else
        {
            ldiv.div_8=cnt_div/64+1; 
            ldiv.div_6=cnt_div/ldiv.div_8;
        }
        return ldiv;
}
static void logic_analizer_ll_set_clock(int sample_rate)
{
    div_68_t ldiv = logic_analizer_ll_convert_sample_rate(sample_rate);
    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = ldiv.div_8; // clk div_8
    I2S0.sample_rate_conf.rx_bck_div_num = ldiv.div_6; // bclk div_6
}
static void logic_analizer_ll_set_pin(int *data_pins,int pin_trigger)
{
    //
    // todo trigger pin
    //

    for (int i = 0; i < 16; i++)
    {
    printf("fun out gpio %d val %lx \n",data_pins[i],GPIO.func_out_sel_cfg[data_pins[i]].val); // test only

    if(data_pins[i]<0) // pin disable - already 0
    {
      gpio_matrix_in(0x30, I2S0I_DATA_IN0_IDX + i, false);  
    }
    else
    {
        
        if(GPIO.func_out_sel_cfg[data_pins[i]].val&0x100)           // if defined as GPIO
        {
            gpio_set_direction(data_pins[i], GPIO_MODE_INPUT_OUTPUT);
        }
        else                                                        // if defined as signal
        {
            PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[i]]);  
        }     
        gpio_matrix_in(data_pins[i], I2S0I_DATA_IN0_IDX + i, false); // connect pin to signal

        // test only
        //        PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[i]]);
        //        gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
        //        gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
        //        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[data_pins[i]], PIN_FUNC_GPIO);
        // end test 

    }
    }
    // cam mode signals must be set to hight    
    gpio_matrix_in(0x38, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);

}
void logic_analizer_ll_config(int *data_pins,int pin_trigger,int sample_rate,la_frame_t *frame)
{
        // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);

    I2S0.conf.rx_start = 0;
    logic_analizer_ll_reset();
    logic_analizer_ll_set_mode();
    logic_analizer_ll_set_clock(sample_rate);
    logic_analizer_ll_set_pin(data_pins,pin_trigger);
    // set dma descriptor
    I2S0.rx_eof_num = frame->fb.len / sizeof(uint32_t);             // count in 32 bit word
    I2S0.in_link.addr = ((uint32_t) & (frame->dma[0])) & 0xfffff;

}
void logic_analizer_ll_start()
{
    I2S0.conf.rx_start = 0;
    I2S_ISR_ENABLE(in_suc_eof);
    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;
}
void IRAM_ATTR logic_analizer_ll_stop()
{
    I2S0.conf.rx_start = 0;
    I2S_ISR_DISABLE(in_suc_eof);
    I2S0.in_link.stop = 1;
}
int logic_analizer_ll_get_sample_rate(int sample_rate)
{
    div_68_t ldiv = logic_analizer_ll_convert_sample_rate(sample_rate);
    return LA_CLK_SAMPLE_RATE/(ldiv.div_6*ldiv.div_8);
}
esp_err_t logic_analizer_ll_init_dma_eof_isr(TaskHandle_t task)
{
    return esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, la_ll_dma_isr, (void *)task, &isr_handle);
}
void logic_analizer_ll_deinit_dma_eof_isr()
{
    esp_intr_free(isr_handle);
}

