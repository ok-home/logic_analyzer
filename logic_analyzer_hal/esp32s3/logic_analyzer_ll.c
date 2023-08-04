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
#include "soc/gdma_struct.h"
#include "soc/gdma_periph.h"
#include "soc/gdma_reg.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"


#include "soc/gpio_sig_map.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#define gpio_matrix_in(a,b,c) esp_rom_gpio_connect_in_signal(a,b,c)
#define gpio_matrix_out(a,b,c,d) esp_rom_gpio_connect_out_signal(a,b,c,d)
#define ets_delay_us(a) esp_rom_delay_us(a)


#if !defined(SOC_GDMA_PAIRS_PER_GROUP) && defined(SOC_GDMA_PAIRS_PER_GROUP_MAX)
#define SOC_GDMA_PAIRS_PER_GROUP SOC_GDMA_PAIRS_PER_GROUP_MAX
#endif

#define TAG "esp32s3 ll"

#include "logic_analyzer_ll.h"

// if define external logic analyzer - define pin as gpio input
// else - self diagnostic analyzer - define pin as defined on firmware + input to i2s

#ifdef CONFIG_ANALYZER_SEPARATE_MODE
#define SEPARATE_MODE_LOGIC_ANALIZER
#else
#undef SEPARATE_MODE_LOGIC_ANALIZER
#endif

#define IN_DONE_DBG 6
#define IN_DSCR_EMPTY_DBG 7
#define IN_EOF_DBG 8 

static void dbg_gpio(void)
{
    gpio_reset_pin(IN_DONE_DBG);
    gpio_set_direction(IN_DONE_DBG,GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN_DSCR_EMPTY_DBG);
    gpio_set_direction(IN_DSCR_EMPTY_DBG,GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN_EOF_DBG);
    gpio_set_direction(IN_EOF_DBG,GPIO_MODE_OUTPUT);
    gpio_set_level(IN_DONE_DBG,0);
    gpio_set_level(IN_DSCR_EMPTY_DBG,0);
    gpio_set_level(IN_EOF_DBG,0);
}

static intr_handle_t isr_handle;
static int dma_num = 0;
//  trigger isr handle
void IRAM_ATTR la_ll_trigger_isr(void *pin)
{
    gpio_matrix_in(0x38, CAM_V_SYNC_IDX, false); // enable cam
    gpio_intr_disable((int)pin);
}
static void IRAM_ATTR la_ll_dma_isr(void *handle)
{
    /* todo  - GDMA_IN_DSCR_EMPTY_CHn_INT: GDMA_IN_DONE_CHn_INT: -> non stop transfer */
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(GDMA.channel[dma_num].in.int_st) status = GDMA.channel[dma_num].in.int_st;
    if (status.val == 0) {
        return;
    }
    GDMA.channel[dma_num].in.int_clr.val = status.val;
    if(status.in_done)
    {
        gpio_set_level(IN_DONE_DBG,1);
        gpio_set_level(IN_DONE_DBG,0);
        //vTaskNotifyGiveFromISR((TaskHandle_t)handle, &HPTaskAwoken);
    }
        if(status.in_dscr_empty)
    {
        gpio_set_level(IN_DSCR_EMPTY_DBG,1);
        gpio_set_level(IN_DSCR_EMPTY_DBG,0);
        //vTaskNotifyGiveFromISR((TaskHandle_t)handle, &HPTaskAwoken);
    }

    if (status.in_suc_eof)
    {
        gpio_set_level(IN_EOF_DBG,1);
        gpio_set_level(IN_EOF_DBG,0);

        vTaskNotifyGiveFromISR((TaskHandle_t)handle, &HPTaskAwoken);
    }
    if (HPTaskAwoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}
static int logic_analyzer_ll_convert_sample_rate(int sample_rate)
{
    return LA_CLK_SAMPLE_RATE/sample_rate;
}
int logic_analyzer_ll_get_sample_rate(int sample_rate)
{
    int  ldiv = logic_analyzer_ll_convert_sample_rate(sample_rate);
    if(ldiv>160) {ldiv=160;}
    return LA_CLK_SAMPLE_RATE / ldiv;
}
static void logic_analyzer_ll_set_clock(int sample_rate)
{
    int ldiv = logic_analyzer_ll_convert_sample_rate(sample_rate);
    if(ldiv>160) // > 1mHz
    {
        ldiv=160;
    }
    // clk out xclk
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CONFIG_ANALYZER_PCLK_PIN], PIN_FUNC_GPIO);
    gpio_set_direction(CONFIG_ANALYZER_PCLK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(CONFIG_ANALYZER_PCLK_PIN, GPIO_FLOATING);
    gpio_matrix_out(CONFIG_ANALYZER_PCLK_PIN, CAM_CLK_IDX, false, false);
    //clk in - pclk
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[CONFIG_ANALYZER_PCLK_PIN]);
    gpio_matrix_in(CONFIG_ANALYZER_PCLK_PIN, CAM_PCLK_IDX, false);
    // Configure clock divider
    LCD_CAM.cam_ctrl.cam_clkm_div_b = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_a = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_num = ldiv;
    LCD_CAM.cam_ctrl.cam_clk_sel = 3;//Select Camera module source clock. 0: no clock. 2: APLL. 3: CLK160. 


    ESP_LOGI(TAG,"ldiv = %d",ldiv);
}

static void logic_analyzer_ll_set_mode(int sample_rate)
{
    /* todo - non stop transfer vh_de_mode, vs_eof_en */
    // attension !! 
    // LCD_CAM.cam_ctrl1.cam_rec_data_bytelen -> logic_analyzer_ll_set_mode  clear len data
    LCD_CAM.cam_ctrl.val = 0;
    LCD_CAM.cam_ctrl1.val = 0;
    LCD_CAM.cam_rgb_yuv.val = 0; // disable converter

    logic_analyzer_ll_set_clock(sample_rate); // set clock divider

    LCD_CAM.cam_ctrl.cam_stop_en = 0;   //Camera stop enable signal, 1: camera stops when GDMA Rx FIFO is full. 0: Do not stop. (R/W)
    LCD_CAM.cam_ctrl.cam_vsync_filter_thres = 0; // Filter by LCD_CAM clock
    LCD_CAM.cam_ctrl.cam_byte_order = 0;  //1: Invert data byte order, only valid in 16-bit mode. 0: Do not change. (R/W)
    LCD_CAM.cam_ctrl.cam_bit_order = 0; //1: Change data bit order, change CAM_DATA_in[7:0] to CAM_DATA_in[0:7] in 8-bit mode, and bits[15:0] to bits[0:15] in 16-bit mode. 0: Do not change. (R/W)
    LCD_CAM.cam_ctrl.cam_line_int_en = 0; //1: Enable to generate LCD_CAM_CAM_HS_INT. 0: Disable. (R/W)
    LCD_CAM.cam_ctrl.cam_vs_eof_en = 0; // 1: Enable CAM_VSYNC to generate in_suc_eof. 0: in_suc_eof is controlled by LCD_CAM_CAM_REC_DATA_BYTELEN. (R/W)

    LCD_CAM.cam_ctrl1.cam_line_int_num = 0; // Configure line number. When the number of received lines reaches this value + 1, LCD_CAM_CAM_HS_INT is triggered. (R/W)
    LCD_CAM.cam_ctrl1.cam_clk_inv = 0; //1: Invert the input signal CAM_PCLK. 0: Do not invert. (R/W)
    LCD_CAM.cam_ctrl1.cam_vsync_filter_en = 0; //1: Enable CAM_VSYNC filter function. 0: Bypass. (R/W)
#ifdef CONFIG_ANALYZER_CHANNEL_NUMBERS_8
    LCD_CAM.cam_ctrl1.cam_2byte_en = 0; // 1: The width of input data is 16 bits. 0: The width of input data is 8 bits. (R/W)
#else
    LCD_CAM.cam_ctrl1.cam_2byte_en = 1; // 1: The width of input data is 16 bits. 0: The width of input data is 8 bits. (R/W)
#endif
    LCD_CAM.cam_ctrl1.cam_de_inv = 0; //CAM_DE invert enable signal, valid in high level. (R/W)
    LCD_CAM.cam_ctrl1.cam_hsync_inv = 0; //CAM_HSYNC invert enable signal, valid in high level. (R/W)
    LCD_CAM.cam_ctrl1.cam_vsync_inv = 0; //CAM_VSYNC invert enable signal, valid in high level. (R/W)
    LCD_CAM.cam_ctrl1.cam_vh_de_mode_en = 0; // 1: Input control signals are CAM_DE and CAM_HSYNC. CAM_VSYNC is 1. 0: Input control signals are CAM_DE and CAM_VSYNC. CAM_HSYNC and CAM_DE are all 1 at the the same time. (R/W)

    LCD_CAM.cam_ctrl.cam_update = 1; //1: Update camera registers. This bit is cleared by hardware. 0: Do not care. (R/W)
    // cam reset, cam fifo reset 
    LCD_CAM.cam_ctrl1.cam_reset = 1;
    LCD_CAM.cam_ctrl1.cam_reset = 0;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 1;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 0;
}

static void logic_analyzer_ll_set_pin(int *data_pins)
{
    /*todo use CAM_H_ENABLE_IDX for control transfer ?*/
    vTaskDelay(5); //??

#ifndef SEPARATE_MODE_LOGIC_ANALIZER

    for (int i = 0; i < LA_MAX_PIN; i++)
    {
        if (data_pins[i] < 0) // pin disable - already 0
        {
            gpio_matrix_in(0x3C, CAM_DATA_IN0_IDX + i, false);
        }
        else
        {
            PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[i]]);
            gpio_matrix_in(data_pins[i], CAM_DATA_IN0_IDX + i, false); // connect pin to signal
        }
    }
#else
    // external not tested ??
    for (int i = 0; i < LA_MAX_PIN; i++)
    {
        if (data_pins[i] < 0) // pin disable - already 0
        {
            gpio_matrix_in(0x3C, CAM_DATA_IN0_IDX + i, false);
        }
        else
        {
            gpio_reset_pin(data_pins[i]);
            gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
            gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
            gpio_matrix_in(data_pins[i], CAM_DATA_IN0_IDX + i, false); // connect pin to signal
        }
    }

#endif

    // CAM_V_SYNC_IDX - stop transfer - set to 0 - set to 1 on start function - other set to 1 enable transfer
    gpio_matrix_in(0x3C, CAM_V_SYNC_IDX, false); //0
    gpio_matrix_in(0x38, CAM_H_SYNC_IDX, false); //1
    gpio_matrix_in(0x38, CAM_H_ENABLE_IDX, false); //1
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
static esp_err_t logic_analyzer_ll_dma_init(void)
{
    for (int x = (SOC_GDMA_PAIRS_PER_GROUP - 1); x >= 0; x--) {
        if (GDMA.channel[x].in.link.addr == 0x0) {
            dma_num = x;
            ESP_LOGI(TAG, "DMA Channel=%d", dma_num);
            break;
        }
        if (x == 0) {
            //cam_deinit();
            ESP_LOGE(TAG, "Can't found available GDMA channel");
			return ESP_FAIL;
        }
    }

    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
    }

    GDMA.channel[dma_num].in.int_clr.val = ~0;
    GDMA.channel[dma_num].in.int_ena.val = 0;

    GDMA.channel[dma_num].in.conf0.val = 0;
    GDMA.channel[dma_num].in.conf1.val = 0;

    GDMA.channel[dma_num].in.conf0.in_rst = 1;
    GDMA.channel[dma_num].in.conf0.in_rst = 0;
    GDMA.channel[dma_num].in.conf0.indscr_burst_en = 1;
    GDMA.channel[dma_num].in.conf0.in_data_burst_en = 1;
    GDMA.channel[dma_num].in.conf1.in_check_owner = 0;
    GDMA.channel[dma_num].in.peri_sel.sel = 5;

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
void logic_analyzer_ll_config(int *data_pins, int sample_rate, la_frame_t *frame)
{
    // Enable and configure cam
    // enable CLK
    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
    }
    logic_analyzer_ll_set_pin(data_pins);
    logic_analyzer_ll_set_mode(sample_rate);
    logic_analyzer_ll_dma_init();


    // set dma descriptor
    GDMA.channel[dma_num].in.link.addr = ((uint32_t) & (frame->dma[0]))&0xfffff;
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = frame->fb.len-1; // count in byte
    LCD_CAM.cam_ctrl.cam_update = 1;
    //ESP_LOGI(TAG,"link s=%d l=%d eof=%d ptr%p next=%lx ",frame->dma[0].size,frame->dma[0].length,frame->dma[0].eof,frame->dma[0].buf,frame->dma[0].empty);
    //ESP_LOGI(TAG,"regcnt=%d framecnt=%d link dma=%x ptr=%p ",LCD_CAM.cam_ctrl1.cam_rec_data_bytelen,frame->fb.len,GDMA.channel[dma_num].in.link.addr,frame->dma);
    // pre start
    GDMA.channel[dma_num].in.int_ena.in_suc_eof = 1;
    GDMA.channel[dma_num].in.int_clr.in_suc_eof = 1;

// DBG
    GDMA.channel[dma_num].in.int_ena.in_done = 1;
    GDMA.channel[dma_num].in.int_clr.in_done = 1;
    GDMA.channel[dma_num].in.int_ena.in_dscr_empty = 1;
    GDMA.channel[dma_num].in.int_clr.in_dscr_empty = 1;
//DBG

    GDMA.channel[dma_num].in.link.stop = 0;
    GDMA.channel[dma_num].in.link.start = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1;                       // enable  transfer

    //ESP_LOGI("TAG","cam start=%d dma start=%d dma stop=%d dma park %d int ena=%d eof=%lx",LCD_CAM.cam_ctrl1.cam_start,GDMA.channel[dma_num].in.link.start,GDMA.channel[dma_num].in.link.stop,GDMA.channel[dma_num].in.link.park,GDMA.channel[dma_num].in.int_ena.in_suc_eof,GDMA.channel[dma_num].in.int_st.val);

}
void logic_analyzer_ll_start()
{
     /*todo use CAM_H_ENABLE_IDX for control transfer ?*/
    gpio_matrix_in(0x38, CAM_V_SYNC_IDX, false); //0
}

void logic_analyzer_ll_triggered_start(int pin_trigger, int trigger_edge)
{
#ifdef CONFIG_ANALYZER_USE_HI_LEVEL5_INTERRUPT
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
    LCD_CAM.cam_ctrl1.cam_start = 0;
    GDMA.channel[dma_num].in.link.stop = 1; // ??
    GDMA.channel[dma_num].in.link.start = 0;
    GDMA.channel[dma_num].in.link.addr = 0x0;
    GDMA.channel[dma_num].in.int_ena.in_suc_eof = 0;
    GDMA.channel[dma_num].in.int_clr.in_suc_eof = 1;

    gpio_reset_pin(CONFIG_ANALYZER_PCLK_PIN);
}

esp_err_t logic_analyzer_ll_init_dma_eof_isr(TaskHandle_t task)
{
	esp_err_t ret = ESP_OK;

    dbg_gpio();

/*    ret = esp_intr_alloc_intrstatus(gdma_periph_signals.groups[0].pairs[dma_num].rx_irq_id,
                                     ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
                                     (uint32_t)&GDMA.channel[dma_num].in.int_st, GDMA_IN_SUC_EOF_CH0_INT_ST_M,
                                     la_ll_dma_isr, (void *)task, &isr_handle);
*/
    ret = esp_intr_alloc(gdma_periph_signals.groups[0].pairs[dma_num].rx_irq_id,
                                     ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
                                     la_ll_dma_isr, (void *)task, &isr_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DMA interrupt allocation of analyzer failed");
		return ret;
	}
    return ret;
}
void logic_analyzer_ll_deinit_dma_eof_isr()
{
    esp_intr_free(isr_handle);
}
