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
#include "hal/spi_ll.h"

#define TAG "esp32c3_ll"

#include "logic_analyzer_ll.h"

#define EOF_CTRL


static intr_handle_t isr_handle;
static int dma_num = 0;

//  trigger isr handle -> start transfer -> slow int 3-4 mks
void IRAM_ATTR la_ll_trigger_isr(void *pin)
{
   GPSPI2.cmd.usr = 1;
   gpio_intr_disable((int)pin);
}
// transfer done -> isr from eof or dma descr_empty
static void IRAM_ATTR la_ll_dma_isr(void *handle)
{
   BaseType_t HPTaskAwoken = pdFALSE;

   typeof(GDMA.intr[dma_num].st) status = GDMA.intr[dma_num].st;
   if (status.val == 0)
   {
      return;
   }
   GDMA.intr[dma_num].clr.val = status.val;
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
// real clock -> use idf spi_ll_master 
int logic_analyzer_ll_get_sample_rate(int sample_rate)
{
   spi_ll_clock_val_t reg_val;
   return spi_ll_master_cal_clock(80000000, sample_rate, 16, &reg_val);
}
// set clock -> use idf spi_ll_master 
static void logic_analyzer_ll_set_clock(int sample_rate)
{
   GPSPI2.clk_gate.clk_en = 1;         // clk enable
   GPSPI2.clk_gate.mst_clk_active = 1; // clk enable
   spi_ll_set_clk_source(&GPSPI2, SPI_CLK_SRC_APB); // clk source 80 mHz - APB_CLK
   spi_ll_master_set_clock(&GPSPI2, 80000000, sample_rate, 16); // set clk for spi
}
// datapin only - no separate mode - no pin to start transfer - transfer ready
static void logic_analyzer_ll_set_pin(int *data_pins, int channels)
{
    //vTaskDelay(5); //??
   if (data_pins[0] >= 0)
   {
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[0]]);
      esp_rom_gpio_connect_in_signal(data_pins[0], spi_periph_signal[SPI2_HOST].spid_in, false); // 0
   }
   if (data_pins[1] >= 0)
   {
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[1]]);
      esp_rom_gpio_connect_in_signal(data_pins[1], spi_periph_signal[SPI2_HOST].spiq_in, false); // 1
   }
   if (data_pins[2] >= 0)
   {
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[2]]);
      esp_rom_gpio_connect_in_signal(data_pins[2], spi_periph_signal[SPI2_HOST].spiwp_in, false); // 2
   }
   if (data_pins[3] >= 0)
   {
      PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[data_pins[3]]);
      esp_rom_gpio_connect_in_signal(data_pins[3], spi_periph_signal[SPI2_HOST].spihd_in, false); // 3
   }
}

// set SPI register - half quad master
static void logic_analyzer_ll_set_mode(int sample_rate, int channels)
{

   logic_analyzer_ll_set_clock(sample_rate);

   GPSPI2.slave.slave_mode = 0; // master
   GPSPI2.user.doutdin = 0;   // half
   GPSPI2.user.usr_miso = 1;  // read
   GPSPI2.ctrl.fread_quad = 1; // 4 lines parralel
   // reset fifo
   GPSPI2.dma_conf.rx_afifo_rst = 1;
   GPSPI2.dma_conf.buf_afifo_rst = 1;
   GPSPI2.dma_conf.dma_afifo_rst = 1;
   GPSPI2.dma_conf.rx_afifo_rst = 0;
   GPSPI2.dma_conf.buf_afifo_rst = 0;
   GPSPI2.dma_conf.dma_afifo_rst = 0;
}

static esp_err_t logic_analyzer_ll_dma_init(void)
{
      // find free dma channel
   for (int x = (SOC_GDMA_PAIRS_PER_GROUP_MAX - 1); x >= 0; x--)
   {
      if (GDMA.channel[x].in.in_link.addr == 0x0)
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
   // enable dma module
   if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN) == 0)
   {
      REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
      REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
      REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
      REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
   }
   // clrear dma interrupt
   GDMA.intr[dma_num].clr.val = ~0;
   GDMA.intr[dma_num].ena.val = 0;
   // default mode
   GDMA.channel[dma_num].in.in_conf0.val = 0;
   GDMA.channel[dma_num].in.in_conf1.val = 0;
   // reset dma
   GDMA.channel[dma_num].in.in_conf0.in_rst = 1;
   GDMA.channel[dma_num].in.in_conf0.in_rst = 0;
   // set burst mode
   GDMA.channel[dma_num].in.in_conf0.indscr_burst_en = 1;
   GDMA.channel[dma_num].in.in_conf0.in_data_burst_en = 1;
   // not check owner on ll descriptor & connect dma channel to GPSPI2
   GDMA.channel[dma_num].in.in_conf1.in_check_owner = 0;
   GDMA.channel[dma_num].in.in_peri_sel.sel = 0; // GPSPI2

   return ESP_OK;
}

void logic_analyzer_ll_config(int *data_pins, int sample_rate, int channels, la_frame_t *frame)
{
   // enable GPSPI2 module
   if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI2_CLK_EN) == 0)
   {
      REG_CLR_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI2_CLK_EN);
      REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI2_CLK_EN);
      REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI2_RST);
      REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI2_RST);
   }

   logic_analyzer_ll_set_pin(data_pins, channels);
   logic_analyzer_ll_dma_init();
   logic_analyzer_ll_set_mode(sample_rate, channels);

   // set dma descriptor
   GDMA.channel[dma_num].in.in_link.addr = ((uint32_t) & (frame->dma[0])) & 0xfffff;
#ifdef EOF_CTRL
   GPSPI2.ms_dlen.ms_data_bitlen = frame->fb.len * 8 - 1; // count in bits
   GPSPI2.dma_conf.rx_eof_en = 1;                         // eof controlled gpspi2
#else
   GPSPI2.ms_dlen.ms_data_bitlen = frame->fb.len * 8 - 1; // eof controlled to DMA linked list -> non stop, ( bytelen = any digit ?? )
   GPSPI2.dma_conf.rx_eof_en = 0;                         // eof controlled DMA linked list
#endif
   //   pre start
   // enably DMA interrupt
   GDMA.intr[dma_num].ena.in_suc_eof = 1;
   GDMA.intr[dma_num].clr.in_suc_eof = 1;
   GDMA.intr[dma_num].ena.in_dscr_empty = 1;
   GDMA.intr[dma_num].clr.in_dscr_empty = 1;
   // enable DMA transfer
   GDMA.channel[dma_num].in.in_link.stop = 0;
   GDMA.channel[dma_num].in.in_link.start = 1; //??
   // update GPSPI2 registers & enable gpspi2->dma transfer ( not start )
   GPSPI2.cmd.update = 1;
   GPSPI2.dma_conf.dma_rx_ena = 1;
}
void logic_analyzer_ll_start()
{
   // start gpspi2->dma transfer once
   GPSPI2.cmd.usr = 1;
}
// sllow interrupt -> gpio, may be redirect current irq 
void logic_analyzer_ll_triggered_start(int pin_trigger, int trigger_edge)
{
   gpio_install_isr_service(0); // default
   gpio_set_intr_type(pin_trigger, trigger_edge);
   gpio_isr_handler_add(pin_trigger, la_ll_trigger_isr, (void *)pin_trigger);
   gpio_intr_disable(pin_trigger);
   gpio_intr_enable(pin_trigger); // start transfer on irq
}
// full stop dma & spi -> todo short command ?
void logic_analyzer_ll_stop()
{
   GPSPI2.dma_conf.dma_rx_ena = 0;

   GDMA.channel[dma_num].in.in_link.stop = 1;
   GDMA.channel[dma_num].in.in_link.start = 0;

   GDMA.intr[dma_num].ena.in_suc_eof = 0;
   GDMA.intr[dma_num].clr.in_suc_eof = 1;
   GDMA.intr[dma_num].ena.in_dscr_empty = 0;
   GDMA.intr[dma_num].clr.in_dscr_empty = 1;

   GDMA.channel[dma_num].in.in_link.addr = 0;

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
