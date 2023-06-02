/* logic analyzer ll example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "logic_analyzer_ll.h"

// if define external logic analyzer - define pin as gpio input
// else - self diagnostic analyzer - define pin as defined on firmware + input to i2s

#ifdef CONFIG_ANALYZER_SEPARATE_MODE
#define SEPARATE_MODE_LOGIC_ANALIZER
#else
#undef SEPARATE_MODE_LOGIC_ANALIZER
#endif

#define gpio_matrix_in(a, b, c) esp_rom_gpio_connect_in_signal(a, b, c)

#define I2S_ISR_ENABLE(i)   \
    {                       \
        I2SX.int_clr.i = 1; \
        I2SX.int_ena.i = 1; \
    }
#define I2S_ISR_DISABLE(i)  \
    {                       \
        I2SX.int_ena.i = 0; \
        I2SX.int_clr.i = 1; \
    }

typedef struct div_68
{
    uint16_t div_8;
    uint16_t div_6;
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
//  trigger isr handle
void IRAM_ATTR la_ll_trigger_isr(void *pin)
{
    gpio_matrix_in(0x38, I2SXI_V_SYNC_IDX, false);
    gpio_intr_disable((int)pin);
}
static void IRAM_ATTR la_ll_dma_isr(void *handle)
{
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(I2SX.int_st) status = I2SX.int_st;
    if (status.val == 0)
    {
        return;
    }
    I2SX.int_clr.val = status.val;
    if (status.in_suc_eof)
    {

        vTaskNotifyGiveFromISR((TaskHandle_t)handle, &HPTaskAwoken);
    }
    if (HPTaskAwoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}
static void logic_analyzer_ll_reset()
{
    I2SX.conf.rx_reset = 1;
    I2SX.conf.rx_reset = 0;
    I2SX.conf.rx_fifo_reset = 1;
    I2SX.conf.rx_fifo_reset = 0;
    I2SX.lc_conf.in_rst = 1;
    I2SX.lc_conf.in_rst = 0;
    I2SX.lc_conf.ahbm_fifo_rst = 1;
    I2SX.lc_conf.ahbm_fifo_rst = 0;
    I2SX.lc_conf.ahbm_rst = 1;
    I2SX.lc_conf.ahbm_rst = 0;
}
static void logic_analyzer_ll_set_mode()
{
    I2SX.conf.val = 0;
    I2SX.conf2.lcd_en = 1;
    I2SX.conf2.camera_en = 1;

    I2SX.fifo_conf.dscr_en = 1;
    I2SX.fifo_conf.rx_fifo_mod = 1;
    I2SX.fifo_conf.rx_fifo_mod_force_en = 1;

    I2SX.conf_chan.rx_chan_mod = 1;
    I2SX.sample_rate_conf.rx_bits_mod = 0;
    I2SX.timing.val = 0;
}
//
// esp32 RefMan - 12.5
// In the LCD mode, the frequency of WS is half of fBCK
// LA_CLK_SAMPLE_RATE = pll160/2 = 80 000 000 hz
//
static div_68_t logic_analyzer_ll_convert_sample_rate(int sample_rate)
{
    div_68_t ret = {
        .div_6 = 1, // div_6 > =2
        .div_8 = 1};

    int delta_div_6 = 0;
    int delta = 0;
    int mindelta = 32767;
    int cnt = LA_CLK_SAMPLE_RATE / sample_rate;
    // extra div div_8+(div_8a/div_8b)
    // int div_8a = 1;
    // int div_8b = 0;
    if (cnt <= 2) // 40 mhz  !! hack !!! in RefMan div6 >=2  ((
    {
        ret.div_6 = 1;
        ret.div_8 = 2;
        return ret;
    }
    if (cnt > 255 * 63)
    {
        ret.div_6 = 63;
        ret.div_8 = 255;
        return ret;
    }
    while (ret.div_6++ < 63)
    {
        ret.div_8 = cnt / ret.div_6;
        if (ret.div_8 > 255 || ret.div_8 == 1)
            continue; // ==1 if div_8>=2 - 20 mHz
        delta = cnt - ret.div_6 * ret.div_8;
        if (delta == 0)
        {
            break;
        }
        if (mindelta > delta)
        {
            mindelta = delta;
            delta_div_6 = ret.div_6;
        }
    }
    if (delta)
    {
        ret.div_6 = delta_div_6;
        ret.div_8 = cnt / ret.div_6;
        //
        // extra div div_8+(div_8a/div_8b)
        //    div_8b = delta;
        //    div_8a = ret.div_6;
        //
    }
    return ret;
}
static void logic_analyzer_ll_set_clock(int sample_rate)
{
    div_68_t ldiv = logic_analyzer_ll_convert_sample_rate(sample_rate);
    // Configure clock divider
    I2SX.clkm_conf.clkm_div_a = 0;
    I2SX.clkm_conf.clkm_div_b = 0;
    I2SX.clkm_conf.clkm_div_num = ldiv.div_8;          // clk div_8
    I2SX.sample_rate_conf.rx_bck_div_num = ldiv.div_6; // bclk div_6
}
static void logic_analyzer_ll_set_pin(int *data_pins)
{

    vTaskDelay(5);
#ifndef SEPARATE_MODE_LOGIC_ANALIZER

    for (int i = 0; i < 16; i++)
    {
        if (data_pins[i] < 0) // pin disable - already 0
        {
            gpio_matrix_in(0x30, I2SXI_DATA_IN0_IDX + i, false);
        }
        else
        {
            PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[i]]);
            gpio_matrix_in(data_pins[i], I2SXI_DATA_IN0_IDX + i, false); // connect pin to signal
        }
    }
#else
    // external not tested ??
    for (int i = 0; i < 16; i++)
    {
        if (data_pins[i] < 0) // pin disable - already 0
        {
            gpio_matrix_in(0x30, I2SXI_DATA_IN0_IDX + i, false);
        }
        else
        {
            gpio_reset_pin(data_pins[i]);
            gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
            gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
            gpio_matrix_in(data_pins[i], I2SXI_DATA_IN0_IDX + i, false); // connect pin to signal
        }
    }

#endif

    // v-sync - stop transfer - set to 0 - set to 1 on start function
    gpio_matrix_in(0x30, I2SXI_V_SYNC_IDX, false);
    // cam mode signals must be set to hight
    gpio_matrix_in(0x38, I2SXI_H_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2SXI_H_ENABLE_IDX, false);
}
void logic_analyzer_ll_config(int *data_pins, int sample_rate, la_frame_t *frame)
{
    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2SX_MODULE);

    I2SX.conf.rx_start = 0;
    logic_analyzer_ll_reset();
    logic_analyzer_ll_set_mode();
    logic_analyzer_ll_set_clock(sample_rate);
    logic_analyzer_ll_set_pin(data_pins);
    // set dma descriptor
    I2SX.rx_eof_num = frame->fb.len / sizeof(uint32_t); // count in 32 bit word
    I2SX.in_link.addr = ((uint32_t) & (frame->dma[0]));
    // pre start
    I2SX.conf.rx_start = 0;
    I2S_ISR_ENABLE(in_suc_eof);
    I2SX.in_link.start = 1;
}
void logic_analyzer_ll_start()
{
    I2SX.conf.rx_start = 1;                        // enable  transfer
    gpio_matrix_in(0x38, I2SXI_V_SYNC_IDX, false); // start transfer
}
void logic_analyzer_ll_triggered_start(int pin_trigger, int trigger_edge)
{
    I2SX.conf.rx_start = 1; // enable transfer
#ifdef CONFIG_ESP_SYSTEM_CHECK_INT_LEVEL_4
    ll_hi_level_triggered_isr_start(pin_trigger, trigger_edge);
#else
    gpio_install_isr_service(0); // default
    gpio_set_intr_type(pin_trigger, trigger_edge);
    gpio_isr_handler_add(pin_trigger, la_ll_trigger_isr, (void *)pin_trigger);
    gpio_intr_disable(pin_trigger);
    gpio_intr_enable(pin_trigger); // start transfer on irq
#endif
}
void logic_analyzer_ll_stop()
{
    I2SX.conf.rx_start = 0;
    I2S_ISR_DISABLE(in_suc_eof);
    I2SX.in_link.stop = 1;
}
int logic_analyzer_ll_get_sample_rate(int sample_rate)
{
    div_68_t ldiv = logic_analyzer_ll_convert_sample_rate(sample_rate);
    return LA_CLK_SAMPLE_RATE / (ldiv.div_6 * ldiv.div_8);
}
esp_err_t logic_analyzer_ll_init_dma_eof_isr(TaskHandle_t task)
{
    return esp_intr_alloc(ETS_I2SX_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, la_ll_dma_isr, (void *)task, &isr_handle);
}
void logic_analyzer_ll_deinit_dma_eof_isr()
{
    esp_intr_free(isr_handle);
}
