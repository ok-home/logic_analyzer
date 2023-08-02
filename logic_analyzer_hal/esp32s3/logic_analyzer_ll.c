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

//#if (ESP_IDF_VERSION_MAJOR >= 5)
#include "soc/gpio_sig_map.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#define gpio_matrix_in(a,b,c) esp_rom_gpio_connect_in_signal(a,b,c)
#define gpio_matrix_out(a,b,c,d) esp_rom_gpio_connect_out_signal(a,b,c,d)
#define ets_delay_us(a) esp_rom_delay_us(a)
//#endif

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


static intr_handle_t isr_handle;
static int dma_num = 0;
//  trigger isr handle
void IRAM_ATTR la_ll_trigger_isr(void *pin)
{
    gpio_matrix_in(0x38, CAM_H_ENABLE_IDX, false); // enable cam
    gpio_intr_disable((int)pin);
}
static void IRAM_ATTR la_ll_dma_isr(void *handle)
{
    BaseType_t HPTaskAwoken = pdFALSE;

    typeof(GDMA.channel[dma_num].in.int_st) status = GDMA.channel[dma_num].in.int_st;
    if (status.val == 0) {
        return;
    }
    GDMA.channel[dma_num].in.int_clr.val = status.val;

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
    LCD_CAM.cam_ctrl1.cam_reset = 1;
    LCD_CAM.cam_ctrl1.cam_reset = 0;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 1;
    LCD_CAM.cam_ctrl1.cam_afifo_reset = 0;
    //GDMA.channel[dma_num].in.conf0.in_rst = 1;
    //GDMA.channel[dma_num].in.conf0.in_rst = 0;

}
static void logic_analyzer_ll_set_mode()
{
    LCD_CAM.cam_ctrl.val = 0;
    LCD_CAM.cam_ctrl1.cam_start = 0; 
    LCD_CAM.cam_ctrl.cam_stop_en = 0;
    //LCD_CAM.cam_ctrl.cam_vsync_filter_thres = 4; // Filter by LCD_CAM clock
    //LCD_CAM.cam_ctrl.cam_update = 0;
    LCD_CAM.cam_ctrl.cam_byte_order = 0;
    LCD_CAM.cam_ctrl.cam_bit_order = 0;
    LCD_CAM.cam_ctrl.cam_line_int_en = 0;
    LCD_CAM.cam_ctrl.cam_vs_eof_en = 0; //1: CAM_VSYNC to generate in_suc_eof. 0: in_suc_eof is controlled by reg_cam_rec_data_cyclelen

    LCD_CAM.cam_ctrl1.val = 0;
    //LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - 1; // Cannot be assigned to 0, and it is easy to overflow
    LCD_CAM.cam_ctrl1.cam_line_int_num = 0; // The number of hsyncs that generate hs interrupts
    LCD_CAM.cam_ctrl1.cam_clk_inv = 0;
    //LCD_CAM.cam_ctrl1.cam_vsync_filter_en = 1;
    LCD_CAM.cam_ctrl1.cam_2byte_en = 1; // 16 bit
    LCD_CAM.cam_ctrl1.cam_de_inv = 0;
    LCD_CAM.cam_ctrl1.cam_hsync_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vsync_inv = 0;
    LCD_CAM.cam_ctrl1.cam_vh_de_mode_en = 1;
    LCD_CAM.cam_ctrl.cam_update = 1;

}
//
// esp32 RefMan - 12.5
// In the LCD mode, the frequency of WS is half of fBCK
// LA_CLK_SAMPLE_RATE = pll160/2 = 80 000 000 hz
//
static int logic_analyzer_ll_convert_sample_rate(int sample_rate)
{
    return 160000000/sample_rate;
}
static void logic_analyzer_ll_set_clock(int sample_rate)
{
    #define PCLK_PIN_TMP 0
    // route clk(pclk) pin ??????
    // clk out xclk
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PCLK_PIN_TMP], PIN_FUNC_GPIO);
    gpio_set_direction(PCLK_PIN_TMP, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_pull_mode(PCLK_PIN_TMP, GPIO_FLOATING);
    gpio_matrix_out(PCLK_PIN_TMP, CAM_CLK_IDX, false, false);
    //clk in - pclk
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[12]);
    gpio_matrix_in(12, CAM_PCLK_IDX, false);

    int ldiv = logic_analyzer_ll_convert_sample_rate(sample_rate);
    // Configure clock divider
    LCD_CAM.cam_ctrl.cam_clkm_div_b = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_a = 0;
    LCD_CAM.cam_ctrl.cam_clkm_div_num = ldiv;
    LCD_CAM.cam_ctrl.cam_clk_sel = 3;//Select Camera module source clock. 0: no clock. 2: APLL. 3: CLK160. 
    ESP_LOGI(TAG,"ldiv = %d",ldiv);
}
static void logic_analyzer_ll_set_pin(int *data_pins)
{

    vTaskDelay(5);

/**/
    //
#ifndef SEPARATE_MODE_LOGIC_ANALIZER

    for (int i = 0; i < 16; i++)
    {
        if (data_pins[i] < 0) // pin disable - already 0
        {
            gpio_matrix_in(0x30, CAM_DATA_IN0_IDX + i, false);
        }
        else
        {
            PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[i]]);
            gpio_matrix_in(data_pins[i], CAM_DATA_IN0_IDX + i, false); // connect pin to signal
        }
    }
#else
    // external not tested ??
    for (int i = 0; i < 16; i++)
    {
        if (data_pins[i] < 0) // pin disable - already 0
        {
            gpio_matrix_in(0x30, CAM_DATA_IN0_IDX + i, false);
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

    // CAM_H_ENABLE - stop transfer - set to 0 - set to 1 on start function - other set to 1 enable transfer
    gpio_matrix_in(0x3C, CAM_V_SYNC_IDX, false); //0
    gpio_matrix_in(0x3C, CAM_H_SYNC_IDX, false); //0
    gpio_matrix_in(0x3C, CAM_H_ENABLE_IDX, false); //0
}

static esp_err_t logic_analyzer_ll_cam_dma_init(void)
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
    GDMA.channel[dma_num].in.conf0.in_rst = 1;
    GDMA.channel[dma_num].in.conf0.in_rst = 0;

    //internal SRAM only
        GDMA.channel[dma_num].in.conf0.indscr_burst_en = 0;
        GDMA.channel[dma_num].in.conf0.in_data_burst_en = 0;

    GDMA.channel[dma_num].in.conf1.in_check_owner = 0;
    // GDMA.channel[cam->dma_num].in.conf1.in_ext_mem_bk_size = 2;

    GDMA.channel[dma_num].in.peri_sel.sel = 5;
    //GDMA.channel[cam->dma_num].in.pri.rx_pri = 1;//rx prio 0-15
    //GDMA.channel[cam->dma_num].in.sram_size.in_size = 6;//This register is used to configure the size of L2 Tx FIFO for Rx channel. 0:16 bytes, 1:24 bytes, 2:32 bytes, 3: 40 bytes, 4: 48 bytes, 5:56 bytes, 6: 64 bytes, 7: 72 bytes, 8: 80 bytes.
    //GDMA.channel[cam->dma_num].in.wight.rx_weight = 7;//The weight of Rx channel 0-15
    return ESP_OK;
}

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

    logic_analyzer_ll_set_clock(sample_rate);
    logic_analyzer_ll_set_pin(data_pins);
    logic_analyzer_ll_set_mode();
    logic_analyzer_ll_reset();

// int ?????

    logic_analyzer_ll_cam_dma_init();


    // set dma descriptor
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = frame->fb.len; // count in byte
    GDMA.channel[dma_num].in.link.addr = ((uint32_t) & (frame->dma[0]));
    ESP_LOGI(TAG,"link s=%d l=%d eof=%d ptr%p next=%lx ",frame->dma[0].size,frame->dma[0].length,frame->dma[0].eof,frame->dma[0].buf,frame->dma[0].empty);
    ESP_LOGI(TAG,"regcnt=%d framecnt=%d link dma=%x ptr=%p ",LCD_CAM.cam_ctrl1.cam_rec_data_bytelen,frame->fb.len,GDMA.channel[dma_num].in.link.addr,frame->dma);
    // pre start
    //GDMA.channel[dma_num].in.link.start = 1;
    //LCD_CAM.cam_ctrl.cam_update = 1;
    //LCD_CAM.cam_ctrl1.cam_start = 1;

    GDMA.channel[dma_num].in.int_ena.in_suc_eof = 1;
    GDMA.channel[dma_num].in.int_clr.in_suc_eof = 1;

}
void logic_analyzer_ll_start()
{
    //LCD_CAM.cam_ctrl.cam_update = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1;                       // enable  transfer
    GDMA.channel[dma_num].in.link.start = 1;
    ESP_LOGI("TAG","cam start=%d dma start=%d int ena=%d eof=%lx",LCD_CAM.cam_ctrl1.cam_start,GDMA.channel[dma_num].in.link.start,GDMA.channel[dma_num].in.int_ena.in_suc_eof,GDMA.channel[dma_num].in.int_st.val);
    gpio_matrix_in(0x38, CAM_V_SYNC_IDX, false); //0
    gpio_matrix_in(0x38, CAM_H_SYNC_IDX, false); //0
    gpio_matrix_in(0x38, CAM_H_ENABLE_IDX, false); //0

    ESP_LOGI(TAG,"regcnt=%d link dma=%x ",LCD_CAM.cam_ctrl1.cam_rec_data_bytelen,GDMA.channel[dma_num].in.link.addr);

}
void logic_analyzer_ll_triggered_start(int pin_trigger, int trigger_edge)
{
    LCD_CAM.cam_ctrl.cam_update = 1;
    LCD_CAM.cam_ctrl1.cam_start = 1;                       // enable  transfer wait CAM_H_ENABLE on int
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
    GDMA.channel[dma_num].in.link.start = 0;
    GDMA.channel[dma_num].in.link.addr = 0x0;

    GDMA.channel[dma_num].in.int_ena.in_suc_eof = 0;
    GDMA.channel[dma_num].in.int_clr.in_suc_eof = 1;
}
int logic_analyzer_ll_get_sample_rate(int sample_rate)
{
    int  ldiv = logic_analyzer_ll_convert_sample_rate(sample_rate);
    return LA_CLK_SAMPLE_RATE / ldiv;
}
esp_err_t logic_analyzer_ll_init_dma_eof_isr(TaskHandle_t task)
{
	esp_err_t ret = ESP_OK;
/*    ret = esp_intr_alloc_intrstatus(gdma_periph_signals.groups[0].pairs[dma_num].rx_irq_id,
                                     ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
                                     (uint32_t)&GDMA.channel[dma_num].in.int_st, GDMA_IN_SUC_EOF_CH0_INT_ST_M,
                                     la_ll_dma_isr, (void *)task, &isr_handle);
*/
    ret = esp_intr_alloc(gdma_periph_signals.groups[0].pairs[dma_num].rx_irq_id,
                                     ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
                                     la_ll_dma_isr, (void *)task, &isr_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DMA interrupt allocation of camera failed");
		return ret;
	}
    ESP_LOGI(TAG,"INTERRUPT");
    return ret;
}
void logic_analyzer_ll_deinit_dma_eof_isr()
{
    esp_intr_free(isr_handle);
}
