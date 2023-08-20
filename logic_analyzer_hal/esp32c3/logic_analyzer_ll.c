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

   for(int i=0;i<frame->fb.len;i++)
   {
      frame->fb.buf[i] = i & 0xff;
   }
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
    //la_trans.length = frame->fb.len*8; //bit
    la_trans.rxlength = frame->fb.len*8;     // bit
    la_trans.rx_buffer = frame->fb.buf;       // Data
    la_trans.flags = SPI_TRANS_MODE_QIO;

   logic_analyzer_ll_set_pin(data_pins,channels);

}
// start transfer without trigger -> v_sync to enable
void logic_analyzer_ll_start()
{
}
// start transfer with trigger -> set irq -> v_sync set to enable on irq handler
void logic_analyzer_ll_triggered_start(int pin_trigger, int trigger_edge)
{
}
// full stop cam, dma, int, pclk, reset pclk pin to default
void logic_analyzer_ll_stop()
{
}

esp_err_t logic_analyzer_ll_init_dma_eof_isr(TaskHandle_t task)
{
   // tmp non interrupt start
   //esp_err_t ret = spi_device_transmit(la_spi, &la_trans); // Transmit!
   //ESP_ERROR_CHECK(ret);               // Should have had no issues.

    spi_transaction_t *ret_trans;

    esp_err_t ret = spi_device_queue_trans(la_spi, &la_trans, portMAX_DELAY);
    if (ret != ESP_OK) return ret;

    ret = spi_device_get_trans_result(la_spi, &ret_trans, portMAX_DELAY);
    if (ret != ESP_OK) return ret;

   xTaskNotifyGive((TaskHandle_t)task);
   return ret;
}
void logic_analyzer_ll_deinit_dma_eof_isr()
{
   spi_bus_remove_device(la_spi);
   spi_bus_free(SPI2_HOST);
}
