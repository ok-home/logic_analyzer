/* logic analyzer ll example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "soc/system_reg.h"
#include "soc/gdma_struct.h"
#include "soc/gdma_periph.h"
#include "soc/gdma_reg.h"

#include "soc/spi_reg.h"

#include "esp_rom_gpio.h"
#include "esp_log.h"

#include "soc/gpio_sig_map.h"
#include "soc/gpio_periph.h"
#include "soc/io_mux_reg.h"
#define gpio_matrix_in(a, b, c) esp_rom_gpio_connect_in_signal(a, b, c)
#define gpio_matrix_out(a, b, c, d) esp_rom_gpio_connect_out_signal(a, b, c, d)

#include "driver/spi_master.h"
#include "soc/spi_periph.h"

#define TAG "esp32c3_ll"

#include "logic_analyzer_ll.h"

static  spi_device_handle_t   la_spi;
static  spi_transaction_t     la_trans;

// if define external logic analyzer - define pin as gpio input
// else - self diagnostic analyzer - define pin as defined on firmware + input to cam

//  trigger isr handle -> start transfer
void IRAM_ATTR la_ll_trigger_isr(void *pin)
{
}
// transfer done -> eof isr from dma descr_empty
static void IRAM_ATTR la_ll_dma_isr(void *handle)
{
}

// sample rate may be not equal to config sample rate -> return real sample rate
int logic_analyzer_ll_get_sample_rate(int sample_rate)
{
   int a_s_rate = 0;
    spi_device_get_actual_freq(la_spi,&a_s_rate);
    return a_s_rate*1000;
}
// set cam pclk, clock & pin.  clock from cam clk or ledclk if clock < 1 MHz
static void logic_analyzer_ll_set_clock(int sample_rate)
{

}
// set cam mode register -> 8/16 bit, eof control from dma,
static void logic_analyzer_ll_set_mode(int sample_rate,int channels)
{
}
// set cam input pin & vsync, hsynk, henable to const to stop transfer
static void logic_analyzer_ll_set_pin(int *data_pins,int channels)
{
      // vTaskDelay(5); //??
      if(data_pins[0]>=0){
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[0]]);
      esp_rom_gpio_connect_in_signal(data_pins[0], spi_periph_signal[SPI2_HOST].spid_in, false);//0
      }
      if(data_pins[1]>=0){
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[1]]);
      esp_rom_gpio_connect_in_signal(data_pins[1], spi_periph_signal[SPI2_HOST].spiq_in, false);//1
      }
      if(data_pins[2]>=0){
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[2]]);
      esp_rom_gpio_connect_in_signal(data_pins[2], spi_periph_signal[SPI2_HOST].spiwp_in, false);//2
      }
      if(data_pins[3]>=0){
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[3]]);
      esp_rom_gpio_connect_in_signal(data_pins[3], spi_periph_signal[SPI2_HOST].spihd_in, false);//3
      }

}
// find free gdma channel, enable dma clock, set dma mode, connect to cam module
static esp_err_t logic_analyzer_ll_dma_init(void)
{
   return ESP_OK;
}
// enable cam module, set cam mode, pin mode, dma mode, dma descr, dma irq
void logic_analyzer_ll_config(int *data_pins, int sample_rate, int channels, la_frame_t *frame)
{
   // debug
   for(int i=0;i<frame->fb.len;i++)
   {
      frame->fb.buf[i] = i & 0xff;
   }
   // debug
    esp_err_t ret;
    memset(&la_spi, 0, sizeof(la_spi)); 
    spi_bus_config_t buscfg = {
        .data0_io_num = -1,
        .data1_io_num = -1,
        .data2_io_num = -1,
        .data3_io_num = -1,     
        .sclk_io_num = -1,
        //.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_OCTAL,
        .max_transfer_sz = frame->fb.len};
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = sample_rate, // Clock out at 1 MHz
        .mode = 0,                         // SPI mode 0
        .spics_io_num = -1,        // CS pin
        .queue_size = 1,                   // We want to be able to queue 7 transactions at a time
        .flags = SPI_DEVICE_TXBIT_LSBFIRST | SPI_DEVICE_HALFDUPLEX,
    };
    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &la_spi);
    ESP_ERROR_CHECK(ret);

    memset(&la_trans, 0, sizeof(la_trans)); // Zero out the transaction
    la_trans.rxlength = frame->fb.len * 8;     // bit
    la_trans.rx_buffer = frame->fb.buf;       // Data
    la_trans.flags = SPI_TRANS_MODE_QIO;

   logic_analyzer_ll_set_pin(data_pins,channels);

}
// start transfer without trigger
void logic_analyzer_ll_start()
{
      // tmp non interrupt start
    spi_transaction_t *ret_trans;
    esp_err_t ret = spi_device_queue_trans(la_spi, &la_trans, portMAX_DELAY);
    if (ret != ESP_OK) return ret;

    ret = spi_device_get_trans_result(la_spi, &ret_trans, portMAX_DELAY);
    if (ret != ESP_OK) return ret;

   xTaskNotifyGive((TaskHandle_t)task);
   return ret;
}
// start transfer with trigger -> set irq -> v_sync set to enable on irq handler
void logic_analyzer_ll_triggered_start(int pin_trigger, int trigger_edge)
{
   // tmp non int start
   logic_analyzer_ll_start();
}
// full stop cam, dma, int, pclk, reset pclk pin to default
void logic_analyzer_ll_stop()
{
   // tmp non int stop
   spi_bus_remove_device(la_spi);
   spi_bus_free(SPI2_HOST);
}

esp_err_t logic_analyzer_ll_init_dma_eof_isr(TaskHandle_t task)
{
   return ESP_OK;
}
void logic_analyzer_ll_deinit_dma_eof_isr()
{
}

#ifdef NON_DRIVER_WORK

static intr_handle_t isr_handle;
static int dma_num = 0;

//  trigger isr handle -> start transfer
void IRAM_ATTR la_ll_trigger_isr(void *pin)
{
    //gpio_matrix_in(0x38, CAM_V_SYNC_IDX, false); // enable transfer
    //gpio_intr_disable((int)pin);
}
// transfer done -> isr from dma descr_empty
static void IRAM_ATTR la_ll_dma_isr(void *handle)
{
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(GDMA.channel[dma_num].in.int_st) status = GDMA.channel[dma_num].in.int_st;
    if (status.val == 0)
    {
        return;
    }
    GDMA.channel[dma_num].in.int_clr.val = status.val;
#ifndef EOF_CTRL
    if (status.in_dscr_empty)
    {
        vTaskNotifyGiveFromISR((TaskHandle_t)handle, &HPTaskAwoken);
    }
#endif
#ifdef EOF_CTRL
    if (status.in_suc_eof)
    {
        vTaskNotifyGiveFromISR((TaskHandle_t)handle, &HPTaskAwoken);
    }
#endif
    if (HPTaskAwoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}
// datapin only - no separate mode - no pin to start transfer - transfer ready
static void logic_analyzer_ll_set_pin(int *data_pins,int channels)
{
      // vTaskDelay(5); //??
      if(data_pins[0]>=0){
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[0]]);
      esp_rom_gpio_connect_in_signal(data_pins[0], spi_periph_signal[SPI2_HOST].spid_in, false);//0
      }
      if(data_pins[1]>=0){
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[1]]);
      esp_rom_gpio_connect_in_signal(data_pins[1], spi_periph_signal[SPI2_HOST].spiq_in, false);//1
      }
      if(data_pins[2]>=0){
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[2]]);
      esp_rom_gpio_connect_in_signal(data_pins[2], spi_periph_signal[SPI2_HOST].spiwp_in, false);//2
      }
      if(data_pins[3]>=0){
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[3]]);
      esp_rom_gpio_connect_in_signal(data_pins[3], spi_periph_signal[SPI2_HOST].spihd_in, false);//3
      }

}

// set cam mode register -> 8/16 bit, eof control from dma,
static void logic_analyzer_ll_set_mode(int sample_rate,int channels)
{
   //GPSPI2

   GSPI2.slave.val = 0;
   GSPI2.slave.slave_mode = 0; // master

   GSPI.cmd.val = 0 ;
   //GSPI.cmd.conf_bitlen = 
   //GSPI.cmd.update = 

   // enable init mode
   GSPI.cmd.usr = 1 ; //before or aftr cfg ??
   GSPI2.slave.usr_conf = 0 ; // single transfer
   
   GSPI2.user.val = 0;
   GSPI2.user.usr_miso = 1; // read
   GSPI2.user.qpi_mode = 1; // 4 lines parralel
   GSPI2.user.usr_dummy = 0;//
   GSPI2.user.usr_addr = 0;//
   GSPI2.user.usr_cmd = 0;//

   //GSPI2.ms_dlen.ms_data_bitlen = 0x0; // transfer len in bits // setup on prestart mode

   GSPI2.ctrl.val = 0 ;
   GSPI2.ctrl.fread_quad = 1; //4 lines parralel

   GSPI2.ms_dlen.ms_data_bitlen = 0x0; // transfer len in bits // setup on prestart mode
   GSPI.cmd.usr = 1 ; before or aftr cfg ?? - start ?? set to 0 ??

   GSPI2.dma_conf.rx_afifo_rst = 1;
   GSPI2.dma_conf.buf_afifo_rst = 1;
   GSPI2.dma_conf.dma_afifo_rst = 1;
   GSPI2.dma_conf.rx_afifo_rst = 0;
   GSPI2.dma_conf.buf_afifo_rst = 0;
   GSPI2.dma_conf.dma_afifo_rst = 0;

   GSPI.cmd.update = 1;

//rx_eof_en
   GSPI2.dma_conf.rx_eof_en = 0 // 1 &
   GSPI2.dma_conf.dma_rx_ena = 1; // start transfer



}

static esp_err_t logic_analyzer_ll_dma_init(void)
{
    for (int x = (SOC_GDMA_PAIRS_PER_GROUP - 1); x >= 0; x--)
    {
        if (GDMA.channel[x].in.link.addr == 0x0)
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

    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN) == 0)
    {
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
#ifdef LA_HW_PSRAM
    GDMA.channel[dma_num].in.conf1.in_ext_mem_bk_size = GDMA_PSRAM_BURST>>5; // 0-> 16 byte burst transfer, 1->32 byte burst transfer
#else
    GDMA.channel[dma_num].in.conf0.indscr_burst_en = 1;
    GDMA.channel[dma_num].in.conf0.in_data_burst_en = 1;
#endif
    GDMA.channel[dma_num].in.conf1.in_check_owner = 0;
    GDMA.channel[dma_num].in.peri_sel.sel = 0; // SPI2

    return ESP_OK;
}

void logic_analyzer_ll_config(int *data_pins, int sample_rate, int channels, la_frame_t *frame)
{
    // Enable and configure cam
    // enable CLK
    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI2_CLK_EN) == 0)
    {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI2_CLK_EN); 
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI2_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI2_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI2_RST);
    }
    logic_analyzer_ll_set_pin(data_pins,channels);
    logic_analyzer_ll_set_mode(sample_rate,channels);
    logic_analyzer_ll_dma_init();

    // set dma descriptor
    GDMA.channel[dma_num].in.link.addr = ((uint32_t) & (frame->dma[0])) & 0xfffff;
#ifdef EOF_CTRL
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = frame->fb.len - 1; // count in byte
#else
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = 64; // eof controlled to DMA linked list cam -> non stop, ( bytelen = any digit )
#endif
    LCD_CAM.cam_ctrl.cam_update = 1;
    //  pre start
    GDMA.channel[dma_num].in.int_ena.in_suc_eof = 1;
    GDMA.channel[dma_num].in.int_clr.in_suc_eof = 1;
    GDMA.channel[dma_num].in.int_ena.in_dscr_empty = 1;
    GDMA.channel[dma_num].in.int_clr.in_dscr_empty = 1;

    GDMA.channel[dma_num].in.link.stop = 0;
    GDMA.channel[dma_num].in.link.start = 1; //??
    //LCD_CAM.cam_ctrl1.cam_start = 1; // enable  transfer
}


esp_err_t logic_analyzer_ll_init_dma_eof_isr(TaskHandle_t task)
{
    esp_err_t ret = ESP_OK;

    ret = esp_intr_alloc(gdma_periph_signals.groups[0].pairs[dma_num].rx_irq_id,
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

#endif