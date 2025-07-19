/* logic analyzer ll example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "soc/system_reg.h"
#include "soc/lcd_cam_struct.h"
#include "soc/lcd_cam_reg.h"
#include "soc/axi_dma_struct.h"
#include "soc/gdma_periph.h"
#include "soc/axi_dma_reg.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"

#include "soc/gpio_sig_map.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#include "soc/hp_sys_clkrst_reg.h"

#include "soc/lcd_cam_struct.h"
#include "soc/dma_pms_struct.h"
#include "esp_cache.h"

#include "esp_etm.h"
#include "driver/gpio_etm.h"

#include "soc/interrupts.h"

#include "soc/gpio_ext_struct.h"
#include "soc/soc_etm_struct.h"

#include "hal/cam_ll.h"

#define gpio_matrix_in(a, b, c) esp_rom_gpio_connect_in_signal(a, b, c)
#define gpio_matrix_out(a, b, c, d) esp_rom_gpio_connect_out_signal(a, b, c, d)

#if !defined(SOC_GDMA_PAIRS_PER_GROUP) && defined(SOC_GDMA_PAIRS_PER_GROUP_MAX)
#define SOC_GDMA_PAIRS_PER_GROUP SOC_GDMA_PAIRS_PER_GROUP_MAX
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
    gpio_matrix_in(63, CAM_V_SYNC_PAD_IN_IDX, false); // enable cam
    // clear GPIO_INT_1 int bit -> disable GPIO_INT_1
    GPIO.pin[(int)pin].int_ena &= ~2;
}
// transfer done -> eof isr from dma descr_empty
static void IRAM_ATTR la_ll_dma_isr(void *handle)
{
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(AXI_DMA.in[dma_num].intr.st) status = AXI_DMA.in[dma_num].intr.st;

    if (status.val == 0)
    {
        return;
    }
    AXI_DMA.in[dma_num].intr.clr.val = status.val;

    if (status.in_dscr_empty_chn_int_st)
    {
        gpio_matrix_in(62, CAM_V_SYNC_PAD_IN_IDX, false); // enable cam
#ifdef CONFIG_ANALYZER_ETM_TRIGGER
        // SOC_ETM.ch_ena_ad0_clr.ch_clr0 = 1; // dis ch0 -> low lewel stop ETM
#endif
        vTaskNotifyGiveFromISR((TaskHandle_t)handle, &HPTaskAwoken);
    }
    else
    {
        ESP_EARLY_LOGI(TAG, "irq status=%x", status.val);
    }

    if (HPTaskAwoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

// sample rate may be not equal to config sample rate -> return real sample rate
int logic_analyzer_ll_get_sample_rate(int sample_rate)
{
    int ldiv = (LA_HW_CLK_SAMPLE_RATE / sample_rate);

    return LA_HW_CLK_SAMPLE_RATE / ldiv;
}
// set cam pclk, clock & pin.  clock from cam clk or ledclk if clock < 1 MHz
static void logic_analyzer_ll_set_clock(int sample_rate)
{

    int ldiv = (LA_HW_CLK_SAMPLE_RATE / sample_rate);
    // clk out xclk -> pclk=clk

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CONFIG_ANALYZER_PCLK_PIN], PIN_FUNC_GPIO);
    gpio_set_direction(CONFIG_ANALYZER_PCLK_PIN, GPIO_MODE_OUTPUT);
    gpio_matrix_out(CONFIG_ANALYZER_PCLK_PIN, CAM_CLK_PAD_OUT_IDX, false, false);

    // input clk pin  -> pclk
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[CONFIG_ANALYZER_PCLK_PIN]);
    gpio_matrix_in(CONFIG_ANALYZER_PCLK_PIN, CAM_PCLK_PAD_IN_IDX, false);
    // enable CLK
    if (HP_SYS_CLKRST.soc_clk_ctrl3.reg_lcdcam_apb_clk_en == 0)
    {
        HP_SYS_CLKRST.soc_clk_ctrl3.reg_lcdcam_apb_clk_en = 1;
        HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_lcdcam = 1;
        HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_lcdcam = 0;
    }

    HP_SYS_CLKRST.ref_clk_ctrl2.reg_ref_160m_clk_en = 1;
    // Configure clock divider
    if (ldiv < 160)
    {
        HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl119, reg_cam_clk_src_sel, 1);
        HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl120, reg_cam_clk_div_num, ldiv - 1);
    }
    else
    {
        HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl119, reg_cam_clk_src_sel, 0);
        HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl120, reg_cam_clk_div_num, ldiv / 4 - 1);
    }
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl120, reg_cam_clk_div_denominator, 0);
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl120, reg_cam_clk_div_numerator, 0);
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl119, reg_cam_clk_en, 1);
}
// set cam mode register -> 8/16 bit, eof control from dma,
static void logic_analyzer_ll_set_mode(int sample_rate, int channels)
{
    // attension !!
    // LCD_CAM.cam_ctrl1.cam_rec_data_bytelen -> logic_analyzer_ll_set_mode  clear len data
    LCD_CAM.cam_ctrl.val = 0;
    LCD_CAM.cam_ctrl1.val = 0;
    LCD_CAM.cam_rgb_yuv.val = 0; // disable converter

    LCD_CAM.cam_ctrl.cam_stop_en = 0;            // Camera stop enable signal, 1: camera stops when GDMA Rx FIFO is full. 0: Do not stop. (R/W)
    LCD_CAM.cam_ctrl.cam_vsync_filter_thres = 0; // Filter by LCD_CAM clock
    LCD_CAM.cam_ctrl.cam_byte_order = 0;         // 1: Invert data byte order, only valid in 16-bit mode. 0: Do not change. (R/W)
    LCD_CAM.cam_ctrl.cam_bit_order = 0;          // 1: Change data bit order, change CAM_DATA_in[7:0] to CAM_DATA_in[0:7] in 8-bit mode, and bits[15:0] to bits[0:15] in 16-bit mode. 0: Do not change. (R/W)
    LCD_CAM.cam_ctrl.cam_line_int_en = 0;        // 1: Enable to generate LCD_CAM_CAM_HS_INT. 0: Disable. (R/W)

    LCD_CAM.cam_ctrl.cam_vs_eof_en = 1; // 1: Enable CAM_VSYNC to generate in_suc_eof. 0: in_suc_eof is controlled by LCD_CAM_CAM_REC_DATA_BYTELEN. (R/W)

    LCD_CAM.cam_ctrl1.cam_line_int_num = 0;    // Configure line number. When the number of received lines reaches this value + 1, LCD_CAM_CAM_HS_INT is triggered. (R/W)
    LCD_CAM.cam_ctrl1.cam_clk_inv = 0;         // 1: Invert the input signal CAM_PCLK. 0: Do not invert. (R/W)
    LCD_CAM.cam_ctrl1.cam_vsync_filter_en = 0; // 1: Enable CAM_VSYNC filter function. 0: Bypass. (R/W)

    if (channels == 8)
    {
        LCD_CAM.cam_ctrl1.cam_2byte_en = 0;
    } // 1: The width of input data is 16 bits. 0: The width of input data is 8 bits. (R/W)
    else
    {
        LCD_CAM.cam_ctrl1.cam_2byte_en = 1;
    } // 1: The width of input data is 16 bits. 0: The width of input data is 8 bits. (R/W)

    LCD_CAM.cam_ctrl1.cam_de_inv = 0;        // CAM_DE invert enable signal, valid in high level. (R/W)
    LCD_CAM.cam_ctrl1.cam_hsync_inv = 0;     // CAM_HSYNC invert enable signal, valid in high level. (R/W)
    LCD_CAM.cam_ctrl1.cam_vsync_inv = 0;     // CAM_VSYNC invert enable signal, valid in high level. (R/W)
    LCD_CAM.cam_ctrl1.cam_vh_de_mode_en = 0; // 1: Input control signals are CAM_DE and CAM_HSYNC. CAM_VSYNC is 1. 0: Input control signals are CAM_DE and CAM_VSYNC. CAM_HSYNC and CAM_DE are all 1 at the the same time. (R/W)

    LCD_CAM.cam_ctrl.cam_update_reg = 1; // 1: Update camera registers. This bit is cleared by hardware. 0: Do not care. (R/W)
    // cam reset, cam fifo reset
    LCD_CAM.cam_ctrl1.cam_reset = 1;
    LCD_CAM.cam_ctrl1.cam_reset = 0;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 1;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 0;
}
// set cam input pin & vsync, hsynk, henable to const to stop transfer
static void logic_analyzer_ll_set_pin(int *data_pins, int channels)
{

#ifndef SEPARATE_MODE_LOGIC_ANALIZER

    for (int i = 0; i < channels; i++)
    {
        if (data_pins[i] < 0) // pin disable - already 0
        {
            gpio_matrix_in(62, CAM_DATA_IN_PAD_IN0_IDX + i, false);
        }
        else
        {
            PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[i]]);
            gpio_matrix_in(data_pins[i], CAM_DATA_IN_PAD_IN0_IDX + i, false); // connect pin to signal
        }
    }
#else
    // external not tested ??
    for (int i = 0; i < channels; i++)
    {
        if (data_pins[i] < 0) // pin disable - already 0
        {
            gpio_matrix_in(62, CAM_DATA_IN_PAD_IN0_IDX + i, false);
        }
        else
        {
            gpio_reset_pin(data_pins[i]);
            gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
            gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
            gpio_matrix_in(data_pins[i], CAM_DATA_IN_PAD_IN0_IDX + i, false); // connect pin to signal
        }
    }
#endif

    // CAM_V_SYNC_IDX - stop transfer - set to 0 - set to 1 on start function - other set to 1 enable transfer
    gpio_matrix_in(62, CAM_V_SYNC_PAD_IN_IDX, false);   // 0
    gpio_matrix_in(63, CAM_H_SYNC_PAD_IN_IDX, false);   // 1
    gpio_matrix_in(63, CAM_H_ENABLE_PAD_IN_IDX, false); // 1
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
1. Configure clock according to Section 29.3.3. Note that in slave mode, the module clock frequency should be two times faster than the PCLK frequency of the image sensor.
2. Configure signal pins according to Table 29-1.
3. Set or clear LCD_CAM_CAM_VH_DE_MODE_EN according to the control signal HSYNC.
4. Set needed RX channel mode and RX data mode, then set the bit LCD_CAM_CAM_UPDATE.
5. Reset RX control unit (Camera_Ctrl) and Async Rx FIFO as described in Section 29.3.4.
6. Enable corresponding interrupts, see Section 29.5.
7. Configure GDMA inlink, and set the length of RX data in LCD_CAM_CAM_REC_DATA_BYTELEN.
8. Start receiving data:
• In master mode, when the slave is ready, set LCD_CAM_CAM_START to start receiving data.
• In slave mode, set LCD_CAM_CAM_START. Receiving data starts after the master provides clock
signal and control signal.
9. Receive data and store the data to the specified address of ESP32-S3 memory. Then corresponding
interrupts set in Step 6 will be generated.
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
    AXI_DMA.in[dma_num].conf.in_peri_sel.peri_in_sel_chn = 0;                         // 0:lcdcam. 1: gpspi_2. 2: gpspi_3. 3: parl_io. 4: aes. 5: sha. 6~15: Dummy
}
// start transfer without trigger -> v_sync to enable
void logic_analyzer_ll_start()
{
    AXI_DMA.in[dma_num].conf.in_link1.inlink_start_chn = 1;
    LCD_CAM.cam_ctrl.cam_update_reg = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1;                  // enable  transfer
    gpio_matrix_in(63, CAM_V_SYNC_PAD_IN_IDX, false); // 1
}
// start transfer with trigger -> set irq -> v_sync set to enable on irq handler
#ifdef CONFIG_ANALYZER_ETM_TRIGGER
static esp_etm_channel_config_t etm_channel_config = {0};
static esp_etm_channel_handle_t etm_channel_handle;
static gpio_etm_event_config_t gpio_etm_event_config = {0};
static esp_etm_event_handle_t gpio_etm_event_handle;
static gpio_etm_task_config_t gpio_etm_task_config = {0};
static esp_etm_task_handle_t gpio_etm_task_handle;
#endif
void logic_analyzer_ll_triggered_start(int pin_trigger, int trigger_edge)
{
#ifdef CONFIG_ANALYZER_ETM_TRIGGER
    //  start ETM
    AXI_DMA.in[dma_num].conf.in_link1.inlink_start_chn = 1;
    LCD_CAM.cam_ctrl.cam_update_reg = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1; // enable  transfer

    gpio_set_level(CONFIG_ANALYZER_ETM_TRIGGER_PIN, 0);
    gpio_set_direction(CONFIG_ANALYZER_ETM_TRIGGER_PIN, GPIO_MODE_INPUT_OUTPUT);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CONFIG_ANALYZER_ETM_TRIGGER_PIN], PIN_FUNC_GPIO);

    esp_err_t etm_err_ret = 0;
    etm_err_ret = esp_etm_new_channel(&etm_channel_config, &etm_channel_handle);

    gpio_etm_event_config.edge = trigger_edge;
    etm_err_ret |= gpio_new_etm_event(&gpio_etm_event_config, &gpio_etm_event_handle);
    etm_err_ret |= gpio_etm_event_bind_gpio(gpio_etm_event_handle, pin_trigger);

    gpio_etm_task_config.action = GPIO_ETM_TASK_ACTION_SET;
    etm_err_ret |= gpio_new_etm_task(&gpio_etm_task_config, &gpio_etm_task_handle);
    etm_err_ret |= gpio_etm_task_add_gpio(gpio_etm_task_handle, CONFIG_ANALYZER_ETM_TRIGGER_PIN); // pin9 ??

    gpio_matrix_in(CONFIG_ANALYZER_ETM_TRIGGER_PIN, CAM_V_SYNC_PAD_IN_IDX, false); // connect task gpio pin to vsync

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
    LCD_CAM.cam_ctrl.cam_update_reg = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1; // enable  transfer

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
    LCD_CAM.cam_ctrl1.cam_start = 0;
    AXI_DMA.in[dma_num].conf.in_link1.inlink_stop_chn = 1;
    AXI_DMA.in[dma_num].conf.in_link1.inlink_start_chn = 0;
    AXI_DMA.in[dma_num].intr.ena.in_dscr_empty_chn_int_ena = 0;
    AXI_DMA.in[dma_num].intr.clr.in_dscr_empty_chn_int_clr = 1;

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
    gpio_set_level(CONFIG_ANALYZER_ETM_TRIGGER_PIN, 0);
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
