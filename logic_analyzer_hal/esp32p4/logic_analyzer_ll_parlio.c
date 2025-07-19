/* logic analyzer ll example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "soc/system_reg.h"
#include "soc/axi_dma_struct.h"
#include "soc/gdma_periph.h"
#include "soc/axi_dma_reg.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"

#include "hal/parlio_types.h"
#include "hal/hal_utils.h"
#include "soc/hp_sys_clkrst_struct.h"
#include "soc/lp_clkrst_struct.h"
#include "soc/parl_io_struct.h"

#include "soc/gpio_sig_map.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#include "soc/hp_sys_clkrst_reg.h"

#include "soc/dma_pms_struct.h"
#include "esp_cache.h"

#include "esp_etm.h"
#include "driver/gpio_etm.h"

#include "soc/interrupts.h"

#include "soc/gpio_ext_struct.h"
#include "soc/soc_etm_struct.h"

#include "hal/parlio_ll.h"

#define gpio_matrix_in(a, b, c) esp_rom_gpio_connect_in_signal(a, b, c)
#define gpio_matrix_out(a, b, c, d) esp_rom_gpio_connect_out_signal(a, b, c, d)

#if !defined(SOC_GDMA_PAIRS_PER_GROUP) && defined(SOC_GDMA_PAIRS_PER_GROUP_MAX)
#define SOC_GDMA_PAIRS_PER_GROUP SOC_GDMA_PAIRS_PER_GROUP_MAX
#endif

#ifdef CONFIG_ANALYZER_USE_LEDC_TIMER_FOR_PCLK
#include "driver/ledc.h"
#endif

#define TAG "esp32p4_ll"

#include "logic_analyzer_ll.h"

// if define external logic analyzer - define pin as gpio input
// else - self diagnostic analyzer - define pin as defined on firmware + input to cam

#ifdef CONFIG_ANALYZER_SEPARATE_MODE
#define SEPARATE_MODE_LOGIC_ANALIZER
#else
#undef SEPARATE_MODE_LOGIC_ANALIZER
#endif

static intr_handle_t isr_handle;
static intr_handle_t gpio_isr_handle;
static int dma_num = 0;
//  trigger isr handle -> start transfer
void IRAM_ATTR la_ll_trigger_isr(void *pin)
{
    gpio_matrix_in(63, PARLIO_RX_DATA15_PAD_IN_IDX, false); // enable transmit
    // clear GPIO_INT_1 int bit -> disable GPIO_INT_1
    GPIO.pin[(int)pin].int_ena &= ~2;
}
// transfer done -> eof isr from dma descr_empty
static void IRAM_ATTR la_ll_dma_isr(void *handle)
{
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(AXI_DMA.in[dma_num].intr.st) status = AXI_DMA.in[dma_num].intr.st;
    ESP_EARLY_LOGE(TAG, "irq status=%x", status.val);

    if (status.val == 0)
    {
        return;
    }
    AXI_DMA.in[dma_num].intr.clr.val = status.val;

    if (status.in_dscr_empty_chn_int_st)
    {
        gpio_matrix_in(62, PARLIO_RX_DATA15_PAD_IN_IDX, false); // stop transmit
#ifdef CONFIG_ANALYZER_ETM_TRIGGER
        //SOC_ETM.ch_ena_ad0_clr.ch_clr0 = 1; // dis ch0 -> low lewel stop ETM
#endif
        vTaskNotifyGiveFromISR((TaskHandle_t)handle, &HPTaskAwoken);
    }
    else
    {
        ESP_EARLY_LOGE(TAG, "irq status=%x", status.val);
    }

    if (HPTaskAwoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

#ifdef CONFIG_ANALYZER_USE_LEDC_TIMER_FOR_PCLK
// for sample rate less then 1 MHz -> use ledc
static void logic_analyzer_ll_set_ledc_pclk(int sample_rate)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = CONFIG_ANALYZER_LEDC_TIMER_NUMBER,
        .duty_resolution = LEDC_TIMER_2_BIT,
        .freq_hz = sample_rate, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = CONFIG_ANALYZER_LEDC_CHANNEL_NUMBER,
        .timer_sel = CONFIG_ANALYZER_LEDC_TIMER_NUMBER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = CONFIG_ANALYZER_PCLK_PIN,
        .duty = 1, // Set duty to 50%
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);
}
#endif
// sample rate may be not equal to config sample rate -> return real sample rate
int logic_analyzer_ll_get_sample_rate(int sample_rate)
{
    int ldiv = (LA_HW_CLK_SAMPLE_RATE / sample_rate);

#ifdef CONFIG_ANALYZER_USE_LEDC_TIMER_FOR_PCLK
    if (ldiv > 160)
    {
        return ((int)ledc_get_freq(LEDC_LOW_SPEED_MODE, CONFIG_ANALYZER_LEDC_TIMER_NUMBER));
    }
#endif
    if (ldiv > 160)
    {
        ldiv = 160;
    }
    return LA_HW_CLK_SAMPLE_RATE / ldiv;
}
// set cam pclk, clock & pin.  clock from cam clk or ledclk if clock < 1 MHz
static void logic_analyzer_ll_set_clock(int sample_rate)
{

    int ldiv = (LA_HW_CLK_SAMPLE_RATE / sample_rate) - 1;
    if (ldiv > 160) // > 1mHz
    {
        ldiv = 160;
    }
    // clk out xclk -> pclk=clk

//    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CONFIG_ANALYZER_PCLK_PIN], PIN_FUNC_GPIO);
//    gpio_set_direction(CONFIG_ANALYZER_PCLK_PIN, GPIO_MODE_OUTPUT);
//    gpio_matrix_out(CONFIG_ANALYZER_PCLK_PIN, PARLIO_RX_CLK_PAD_OUT_IDX, false, false);


#ifdef CONFIG_ANALYZER_USE_LEDC_TIMER_FOR_PCLK
    if ((LA_HW_CLK_SAMPLE_RATE / sample_rate) > 160)
    {
        ldiv = 8; // cam clk to 20 MHz
        logic_analyzer_ll_set_ledc_pclk(sample_rate);
        logic_analyzer_ll_set_ledc_pclk(sample_rate);
    }
#endif
    // input clk pin  -> pclk
//    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[CONFIG_ANALYZER_PCLK_PIN]);
//    gpio_matrix_in(CONFIG_ANALYZER_PCLK_PIN, PARLIO_RX_CLK_PAD_IN_IDX, false);
    // enable CLK
/*
//    if (HP_SYS_CLKRST.soc_clk_ctrl2.reg_parlio_apb_clk_en == 0)
//    {
        HP_SYS_CLKRST.soc_clk_ctrl1.reg_parlio_sys_clk_en = 1;
        HP_SYS_CLKRST.soc_clk_ctrl2.reg_parlio_apb_clk_en = 1;
//    }
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio = 1;
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio = 0;

    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio_rx = 1;
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio_rx = 0;

    HP_SYS_CLKRST.ref_clk_ctrl2.reg_ref_160m_clk_en = 1;
    HP_SYS_CLKRST.ref_clk_ctrl2.reg_tm_160m_clk_en = 1;
    
    // Configure clock divider
    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_src_sel = 2; // 2-PLL160

    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl117, reg_parlio_rx_clk_div_num, ldiv);
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl118, reg_parlio_rx_clk_div_denominator,0);
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl118, reg_parlio_rx_clk_div_numerator,0);
// on or off ??
    LP_AON_CLKRST.hp_clk_ctrl.hp_pad_parlio_rx_clk_en = 1;
    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_en = 1;
*/

    HP_SYS_CLKRST.ref_clk_ctrl2.reg_ref_160m_clk_en = 1;
    HP_SYS_CLKRST.soc_clk_ctrl1.reg_parlio_sys_clk_en = 1;
    HP_SYS_CLKRST.soc_clk_ctrl2.reg_parlio_apb_clk_en = 1;

    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_src_sel = 2; // 2-PLL160
    //rx
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl117, reg_parlio_rx_clk_div_num,ldiv); // 160/8
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl118, reg_parlio_rx_clk_div_denominator,0);
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl118, reg_parlio_rx_clk_div_numerator,0);

    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio = 1;
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio = 0;
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio_rx = 1;
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio_rx = 0;

    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_en = 0;


}
// set cam mode register -> 8/16 bit, eof control from dma,
static void logic_analyzer_ll_set_mode(int sample_rate, int channels)
{

    PARL_IO.clk.clk_en = 0 ; // ??
    PARL_IO.rx_genrl_cfg.rx_gating_en = 0;
    PARL_IO.rx_mode_cfg.rx_sw_en = 0; // 1-soft start mode
    PARL_IO.rx_mode_cfg.rx_smp_mode_sel = 0; //0 - lvl
    PARL_IO.rx_mode_cfg.rx_ext_en_sel = 0xf; // datapin 15
    PARL_IO.rx_genrl_cfg.rx_eof_gen_sel = 1; //bit lenght eof - any

    if (channels == 8)
        {PARL_IO.rx_data_cfg.rx_bus_wid_sel = 3;} //8 bit 
    else 
        {PARL_IO.rx_data_cfg.rx_bus_wid_sel = 4;} //16 bit

        PARL_IO.rx_start_cfg.rx_start = 0;
        PARL_IO.fifo_cfg.rx_fifo_srst = 1; // reset fifo
        PARL_IO.fifo_cfg.rx_fifo_srst = 0; // reset fifo

/*
    PARL_IO.rx_mode_cfg.rx_ext_en_sel = 0xf; //data15 as ext en signal
    PARL_IO.rx_mode_cfg.rx_smp_mode_sel = 2; //ext level mode
    PARL_IO.rx_mode_cfg.rx_sw_en = 1; // 1-soft start mode
    PARL_IO.rx_mode_cfg.rx_ext_en_inv = 0; //
    PARL_IO.rx_mode_cfg.rx_pulse_submode_sel = 0; // any val  on ext level mode

    PARL_IO.rx_data_cfg.rx_bitlen = 0XFFFF; // any on ext level mode
    PARL_IO.rx_data_cfg.rx_data_order_inv = 0; // bit order
    if (channels == 8)
        {PARL_IO.rx_data_cfg.rx_bus_wid_sel = 3;} //8 bit 
    else 
        {PARL_IO.rx_data_cfg.rx_bus_wid_sel = 4;} //16 bit

    PARL_IO.rx_genrl_cfg.rx_eof_gen_sel = 0; // eof generate ext level signal
    PARL_IO.rx_genrl_cfg.rx_gating_en = 0; // no clock gate
    PARL_IO.rx_genrl_cfg.rx_timeout_en = 0; // timeout disable
    PARL_IO.rx_genrl_cfg.rx_timeout_thres = 0;//

    PARL_IO.fifo_cfg.rx_fifo_srst = 1; // reset fifo
    PARL_IO.fifo_cfg.rx_fifo_srst = 0; // reset fifo

    PARL_IO.int_ena.val = 0; //disable int
    PARL_IO.clk.clk_en = 1 ; // ??
    PARL_IO.rx_start_cfg.rx_start = 0;
    PARL_IO.reg_update.rx_reg_update = 1;
*/


}
// set cam input pin & vsync, hsynk, henable to const to stop transfer
static void logic_analyzer_ll_set_pin(int *data_pins, int channels)
{

#ifndef SEPARATE_MODE_LOGIC_ANALIZER

    for (int i = 0; i < channels; i++)
    {
        if (data_pins[i] < 0) // pin disable - already 0
        {
            gpio_matrix_in(62, PARLIO_RX_DATA0_PAD_IN_IDX + i, false);
        }
        else
        {
            PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[i]]);
            gpio_matrix_in(data_pins[i], PARLIO_RX_DATA0_PAD_IN_IDX + i, false); // connect pin to signal
        }
    }
#else
    // external not tested ??
    for (int i = 0; i < channels; i++)
    {
        if (data_pins[i] < 0) // pin disable - already 0
        {
            gpio_matrix_in(62, PARLIO_RX_DATA0_PAD_IN_IDX + i, false);
        }
        else
        {
            gpio_reset_pin(data_pins[i]);
            gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
            gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
            gpio_matrix_in(data_pins[i], PARLIO_RX_DATA0_PAD_IN_IDX + i, false); // connect pin to signal
        }
    }
#endif

    // stop transfer - set to 0 - set to 1 on start function - other set to 1 enable transfer
    //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CONFIG_ANALYZER_ETM_TRIGGER_PIN], PIN_FUNC_GPIO);
    //PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[CONFIG_ANALYZER_ETM_TRIGGER_PIN]);
    //gpio_matrix_in(CONFIG_ANALYZER_ETM_TRIGGER_PIN, PARLIO_RX_DATA15_PAD_IN_IDX, false);

    gpio_matrix_in(62, PARLIO_RX_DATA15_PAD_IN_IDX, false);   // 0
}
/*
1. Set GDMA_IN_RST_CHn first to 1 and then to 0, to reset the state machine of GDMA’s receive channel and FIFO pointer;
2. Load an inlink, and configure GDMA_INLINK_ADDR_CHn with address of the first receive descriptor;
3. Configure GDMA_PERI_IN_SEL_CHn with the value corresponding to the peripheral to be connected, as shown in Table 3-1;
4. Set GDMA_INLINK_START_CHn to enable GDMA’s receive channel for data transfer;
5. Configure and enable the corresponding peripheral (SPI2, SPI3, UHCI0 (UART0, UART1, or UART2), I2S0,I2S1, AES, SHA, and ADC). See details in individual chapters of these peripherals;
6. Wait for GDMA_IN_SUC_EOF_CHn_INT interrupt, which indicates that a data frame or packet has been
received.
*/
// find free gdma channel, enable dma clock, set dma mode, connect to cam module
static esp_err_t logic_analyzer_ll_dma_init(void)
{
    for (int x = (SOC_GDMA_PAIRS_PER_GROUP - 1); x >= 0; x--)
    {
        if (AXI_DMA.in[x].conf.in_link2.inlink_addr_chn == 0x0)
        {
            dma_num = x;
            break;
        }
        if (x == 0)
        {
            // cam_deinit();
            ESP_LOGE(TAG, "Can't found available GDMA channel");
            return ESP_FAIL;
        }
    }
    if (HP_SYS_CLKRST.soc_clk_ctrl1.reg_ahb_pdma_sys_clk_en == 0)
    {
        HP_SYS_CLKRST.soc_clk_ctrl1.reg_ahb_pdma_sys_clk_en = 1;
    }
    if (HP_SYS_CLKRST.soc_clk_ctrl1.reg_axi_pdma_sys_clk_en == 0)
    {
        HP_SYS_CLKRST.soc_clk_ctrl1.reg_axi_pdma_sys_clk_en = 1;
        HP_SYS_CLKRST.hp_rst_en1.reg_rst_en_axi_pdma = 1;
        HP_SYS_CLKRST.hp_rst_en1.reg_rst_en_axi_pdma = 0;
    }

    AXI_DMA.in[dma_num].intr.clr.val = ~0;
    AXI_DMA.in[dma_num].intr.ena.val = 0;

    AXI_DMA.in[dma_num].conf.in_conf0.val = 0;
    AXI_DMA.in[dma_num].conf.in_conf1.val = 0;

    AXI_DMA.in[dma_num].conf.in_conf0.in_rst_chn = 1;
    AXI_DMA.in[dma_num].conf.in_conf0.in_rst_chn = 0;

    AXI_DMA.in[dma_num].conf.in_conf0.in_burst_size_sel_chn = GDMA_BURST_CONST;
    AXI_DMA.in[dma_num].conf.in_conf0.indscr_burst_en_chn = 0;

    AXI_DMA.in[dma_num].conf.in_conf1.in_check_owner_chn = 1;

    // gdma arbiter & priority -> not used
    // AXI_DMA.in[dma_num].conf.in_pri.rx_pri_chn = 5; //0-5
    // AXI_DMA.in[dma_num].conf.in_pri.rx_ch_arb_weigh_chn = 15; //0-15
    // AXI_DMA.arb_timeout.arb_timeout_rx = 256;
    // AXI_DMA.in[dma_num].conf.in_pri.rx_arb_weigh_opt_dir_chn = 1; //

    return ESP_OK;
}
/*
*/
// enable cam module, set cam mode, pin mode, dma mode, dma descr, dma irq
void logic_analyzer_ll_config(int *data_pins, int sample_rate, int channels, la_frame_t *frame)
{
    // Enable and configure cam

    logic_analyzer_ll_set_clock(sample_rate); // set clock divider
    logic_analyzer_ll_set_pin(data_pins, channels);
    logic_analyzer_ll_set_mode(sample_rate, channels);
    logic_analyzer_ll_dma_init();

    // LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = 64; // eof controlled to DMA linked list cam -> non stop, ( bytelen = any digit )

    //  pre start
    AXI_DMA.in[dma_num].intr.ena.in_dscr_empty_chn_int_ena = 1;
    AXI_DMA.in[dma_num].intr.clr.in_dscr_empty_chn_int_clr = 1;

    AXI_DMA.in[dma_num].conf.in_conf0.in_rst_chn = 1;
    AXI_DMA.in[dma_num].conf.in_conf0.in_rst_chn = 0;
    AXI_DMA.in[dma_num].conf.in_link2.inlink_addr_chn = ((uint32_t)&(frame->dma[0])); // set dma descriptor
    AXI_DMA.in[dma_num].conf.in_peri_sel.peri_in_sel_chn = 3; // 0:lcdcam. 1: gpspi_2. 2: gpspi_3. 3: parl_io. 4: aes. 5: sha. 6~15: Dummy
}
// start transfer without trigger -> v_sync to enable
void logic_analyzer_ll_start()
{
    AXI_DMA.in[dma_num].conf.in_link1.inlink_start_chn = 1;
    PARL_IO.reg_update.rx_reg_update = 1;
    PARL_IO.rx_start_cfg.rx_start = 1;
    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_en = 1;

    gpio_matrix_in(63, PARLIO_RX_DATA15_PAD_IN_IDX, false); // 1
}
// start transfer with trigger -> set irq -> v_sync set to enable on irq handler
#ifdef CONFIG_ANALYZER_ETM_TRIGGER
 static esp_etm_channel_config_t etm_channel_config = {0};
 static esp_etm_channel_handle_t etm_channel_handle;
 static gpio_etm_event_config_t gpio_etm_event_config = {0};
 static esp_etm_event_handle_t gpio_etm_event_handle;
 static gpio_etm_task_config_t gpio_etm_task_config = {0};
 static  esp_etm_task_handle_t gpio_etm_task_handle;
#endif
void logic_analyzer_ll_triggered_start(int pin_trigger, int trigger_edge)
{
#ifdef CONFIG_ANALYZER_ETM_TRIGGER
    //  start ETM
    AXI_DMA.in[dma_num].conf.in_link1.inlink_start_chn = 1;
    PARL_IO.reg_update.rx_reg_update = 1;
    PARL_IO.rx_start_cfg.rx_start = 1;
    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_en = 1;

    gpio_set_level(CONFIG_ANALYZER_ETM_TRIGGER_PIN,0);
    gpio_set_direction(CONFIG_ANALYZER_ETM_TRIGGER_PIN, GPIO_MODE_INPUT_OUTPUT);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CONFIG_ANALYZER_ETM_TRIGGER_PIN], PIN_FUNC_GPIO);

    esp_err_t etm_err_ret = 0;
    etm_err_ret = esp_etm_new_channel(&etm_channel_config, &etm_channel_handle);

    gpio_etm_event_config.edge = trigger_edge;
    etm_err_ret |= gpio_new_etm_event(&gpio_etm_event_config, &gpio_etm_event_handle);
    etm_err_ret |= gpio_etm_event_bind_gpio(gpio_etm_event_handle, pin_trigger);

    gpio_etm_task_config.action = GPIO_ETM_TASK_ACTION_SET;
    etm_err_ret |= gpio_new_etm_task(&gpio_etm_task_config,&gpio_etm_task_handle);
    etm_err_ret |= gpio_etm_task_add_gpio(gpio_etm_task_handle, CONFIG_ANALYZER_ETM_TRIGGER_PIN);// pin9 ??

    gpio_matrix_in(CONFIG_ANALYZER_ETM_TRIGGER_PIN, PARLIO_RX_DATA15_PAD_IN_IDX, false); // connect task gpio pin to vsync

    etm_err_ret |= esp_etm_channel_connect(etm_channel_handle, gpio_etm_event_handle, gpio_etm_task_handle);
    etm_err_ret |= esp_etm_channel_enable(etm_channel_handle);

/*
    // low level  etm with gdma starter - not used
    // enable clock
    HP_SYS_CLKRST.soc_clk_ctrl1.reg_etm_sys_clk_en = 1;
    HP_SYS_CLKRST.soc_clk_ctrl3.reg_etm_apb_clk_en = 1;
    HP_SYS_CLKRST.hp_rst_en1.reg_rst_en_etm = 1;
    HP_SYS_CLKRST.hp_rst_en1.reg_rst_en_etm = 0;

    SOC_ETM.ch_ena_ad0_clr.ch_clr0 = 1; // dis ch0
    // conn gpio pin to gpio etm channel 0-7 //todo find free ch
    GPIO_ETM.etm_event_chn_cfg[0].etm_chn_event_sel = pin_trigger;
    GPIO_ETM.etm_event_chn_cfg[0].etm_chn_event_en = 1;

    SOC_ETM.channel[0].eid.evt_id = 1;    // 9; //gpio ch0 rising on etm ch 0
    SOC_ETM.channel[0].tid.task_id = 193; // gdma axi ch2 start

    AXI_DMA.in[dma_num].conf.in_conf0.in_etm_en_chn = 1;
    LCD_CAM.cam_ctrl.cam_update_reg = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1;                  // enable  transfer
    gpio_matrix_in(63, CAM_V_SYNC_PAD_IN_IDX, false); // enable cam_vs

    SOC_ETM.ch_ena_ad0_set.ch_set0 = 1; // ena ch0
*/
#else
    AXI_DMA.in[dma_num].conf.in_link1.inlink_start_chn = 1;
    PARL_IO.reg_update.rx_reg_update = 1;
    PARL_IO.rx_start_cfg.rx_start = 1;
    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_en = 1;

    esp_err_t ret = esp_intr_alloc(ETS_GPIO_INTR1_SOURCE, ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM, la_ll_trigger_isr, (void *)pin_trigger, &gpio_isr_handle);
    if (ret)
    {
        ESP_LOGE(TAG, "GPIO_INT_1 intr alloc fail error=%x capture on non triggered mode", ret);
        logic_analyzer_ll_start();
    }
    else
    {
        if (GPIO.pin[pin_trigger].int_ena == 0) // pin not used on other IRQ, set trigger edge
        {
            GPIO.pin[pin_trigger].int_type = trigger_edge;
        }
        // clear intr status
        // todo intr1 status ??
        if (pin_trigger < 32)
        {
            GPIO.status_w1tc.val = (0x1 << pin_trigger);
        }
        else
        {
            GPIO.status1_w1tc.val = (0x1 << (pin_trigger - 32));
        }
        GPIO.pin[pin_trigger].int_ena |= 2; // enable GPIO_INT_1 intr
    }

#endif
}
// full stop cam, dma, int, pclk, reset pclk pin to default
void logic_analyzer_ll_stop()
{
    AXI_DMA.in[dma_num].conf.in_link2.inlink_addr_chn = 0x0;
    PARL_IO.rx_start_cfg.rx_start = 0;
    AXI_DMA.in[dma_num].conf.in_link1.inlink_stop_chn = 1;
    AXI_DMA.in[dma_num].conf.in_link1.inlink_start_chn = 0;
    AXI_DMA.in[dma_num].intr.ena.in_dscr_empty_chn_int_ena = 0;
    AXI_DMA.in[dma_num].intr.clr.in_dscr_empty_chn_int_clr = 1;

#ifdef CONFIG_ANALYZER_USE_LEDC_TIMER_FOR_PCLK
    ledc_stop(LEDC_LOW_SPEED_MODE, CONFIG_ANALYZER_LEDC_CHANNEL_NUMBER, 0);
#endif
    gpio_set_direction(CONFIG_ANALYZER_PCLK_PIN, GPIO_MODE_DISABLE);

    if (gpio_isr_handle)
    {
        esp_intr_free(gpio_isr_handle);
        gpio_isr_handle = NULL;
    }
#ifdef CONFIG_ANALYZER_ETM_TRIGGER
      esp_etm_channel_disable(etm_channel_handle);
      gpio_etm_task_rm_gpio(gpio_etm_task_handle, CONFIG_ANALYZER_ETM_TRIGGER_PIN);
      esp_etm_del_task(gpio_etm_task_handle);
      esp_etm_del_event(gpio_etm_event_handle);
      esp_etm_del_channel(etm_channel_handle);
      gpio_set_level(CONFIG_ANALYZER_ETM_TRIGGER_PIN,0);
#endif
}

esp_err_t logic_analyzer_ll_init_dma_eof_isr(TaskHandle_t task)
{
    esp_err_t ret = ESP_OK;

    /*    ret = esp_intr_alloc_intrstatus(gdma_periph_signals.groups[0].pairs[dma_num].rx_irq_id,
                                         ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
                                         (uint32_t)&AXI_DMA.channel[dma_num].in.int_st, GDMA_IN_SUC_EOF_CH0_INT_ST_M,
                                         la_ll_dma_isr, (void *)task, &isr_handle);
    */
    ret = esp_intr_alloc(gdma_periph_signals.groups[1].pairs[dma_num].rx_irq_id,
                         ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
                         la_ll_dma_isr, (void *)task, &isr_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "DMA interrupt allocation of analyzer failed");
        return ret;
    }
    return ret;
}
void logic_analyzer_ll_deinit_dma_eof_isr()
{
    esp_intr_free(isr_handle);
}
