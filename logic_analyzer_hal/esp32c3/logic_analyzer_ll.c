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

// #include "driver/spi_master.h"
#include "soc/spi_periph.h"
#include "hal/spi_ll.h"
#include "soc/gpio_struct.h"

#define TAG "esp32c3_ll"

#include "logic_analyzer_ll.h"

static intr_handle_t isr_handle;
static intr_handle_t gpio_isr_handle;
static int dma_num = 0;

//#define HI_LEVEL_INT_RISCV 1
#ifdef HI_LEVEL_INT_RISCV
#include "riscv/rv_utils.h"
// tmp hi_lvl_int
typedef union // opcode JAL instruction struct
{
   uint32_t val;
   struct { 
      uint32_t opcode:7;
      uint32_t regs:5;
      uint32_t b_12_19:8;
      uint32_t b_11:1;
      uint32_t b_1_10:10;
      uint32_t b_20;
   };
} opcode_t;

extern uint32_t _vector_table[32]; // swap to DRAM adress !!!!
static uint32_t *_vector_table_to_write; // address to write VT
static int hi_level_trigger_pin = -1;
static uint32_t hi_level_backup_ivect_data = 0;
static uint32_t hi_level_ivect_idx = 0;
// GCC compile __attribute__((interrupt)), - code save & restore used register in stack & mret instruction on return
__attribute__((interrupt))
void IRAM_ATTR la_hi_level_ll_trigger_isr(void)
{
   if(GPIO.status.intr_st & (0x1<<hi_level_trigger_pin))
   {
   GPSPI2.cmd.usr = 1; // start gdma
   GPIO.pin[hi_level_trigger_pin].int_ena &= ~2; // clear nmi int bit -> disable nmi
   _vector_table_to_write[hi_level_ivect_idx] = hi_level_backup_ivect_data; // restore _vector_table to default
   }
}
   // change jal instruction to hi_lvl_irq_handler -> hack
   // default handler -> _interrupt_handler -> jamp to ESP IDF IRQ dispatcher and irq call la_ll_trigger_isr
   // hack -> change jal instruction in _vector_table to (j la_hi_level_ll_trigger_isr), it works without ESP IDF IRQ dispatcher - directly
   // irq handler change from default handler allocated on esp_intr_alloc
   // la_hi_level_ll_trigger_isr -> compiled with __attribute__((interrupt)) - code save & restore used register & mret instruction on return
   // calculate offset to la_hi_level_ll_trigger_isr
[[gnu::optimize("-O0")]]
void la_hi_level_int_enable(int pin_trigger)
{
   hi_level_trigger_pin = pin_trigger; // trigg_pin to hi_lvl_irq_handler
   hi_level_ivect_idx =  esp_intr_get_intno(gpio_isr_handle); // idx of handler in _vector_table
   hi_level_backup_ivect_data =  _vector_table[hi_level_ivect_idx]; // backup default handler in _vector_table

   int diff = (int)((uint8_t *)la_hi_level_ll_trigger_isr-(uint8_t*)&_vector_table[hi_level_ivect_idx]);
   opcode_t opcode; // create JAL x0 instruction
   opcode.val = 0; 
   opcode.b_20 = diff>>20;
   opcode.b_12_19 = diff>>12;
   opcode.b_11 = diff>>11;
   opcode.b_1_10 = diff>>1;
   opcode.opcode = 0x6f;
   opcode.regs=0;

   _vector_table_to_write = &_vector_table[0]-0x1c0000; // mapped ivect table to DRAM
   _vector_table_to_write[hi_level_ivect_idx] = opcode.val; // change irq handler addr in ivect table
}

#endif
//  trigger isr handle -> start transfer -> slow int 3-4 mks
void IRAM_ATTR la_ll_trigger_isr(void *pin)
{
   GPSPI2.cmd.usr = 1;
   GPIO.pin[(int)pin].int_ena &= ~2; // clear nmi int bit -> disable nmi
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

   if (status.in_suc_eof)
   {
      vTaskNotifyGiveFromISR((TaskHandle_t)handle, &HPTaskAwoken);
   }

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
   GPSPI2.clk_gate.clk_en = 1;                                  // clk enable
   GPSPI2.clk_gate.mst_clk_active = 1;                          // clk enable
   spi_ll_set_clk_source(&GPSPI2, SPI_CLK_SRC_APB);             // clk source 80 mHz - APB_CLK
   spi_ll_master_set_clock(&GPSPI2, 80000000, sample_rate, 16); // set clk for spi
}
// datapin only - no separate mode - no pin to start transfer - transfer ready
static void logic_analyzer_ll_set_pin(int *data_pins, int channels)
{
   // vTaskDelay(5); //??
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

   GPSPI2.slave.slave_mode = 0;  // master
   GPSPI2.user.doutdin = 0;      // half
   GPSPI2.user.usr_miso = 1;     // read
   GPSPI2.ctrl.fread_quad = 1;   // 4 lines parralel
   GPSPI2.ctrl.rd_bit_order = 1; // LSB first
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
   gpio_isr_handle = NULL;
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

   GPSPI2.ms_dlen.ms_data_bitlen = frame->fb.len * 8 - 1; // count in bits
   GPSPI2.dma_conf.rx_eof_en = 1;                         // eof controlled gpspi2

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
// slow interrupt -> gpio, may be redirect current irq
// use NMI GPIO irq source -> Work with level irq ??? or not use GPIO.status_w1tc ???
void logic_analyzer_ll_triggered_start(int pin_trigger, int trigger_edge)
{
   esp_err_t ret = esp_intr_alloc(ETS_GPIO_NMI_SOURCE, ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM , la_ll_trigger_isr, (void *)pin_trigger, &gpio_isr_handle);
   if (ret)
   {
      ESP_LOGE(TAG, "NMI intr alloc fail error=%x capture on non triggered mode", ret);
      logic_analyzer_ll_start();
   }
   else
   {
      if (GPIO.pin[pin_trigger].int_ena == 0) // pin not used on other IRQ, set trigger edge
      {
         GPIO.pin[pin_trigger].int_type = trigger_edge;
      }
#ifdef HI_LEVEL_INT_RISCV
   la_hi_level_int_enable(pin_trigger);
#endif
      GPIO.status_w1tc.val = (0x1 << pin_trigger); // clear intr status
      GPIO.pin[pin_trigger].int_ena |= 2;        // enable nmi intr
   }
}
// full stop dma & spi -> todo short command ?
void logic_analyzer_ll_stop()
{
   GPSPI2.dma_conf.dma_rx_ena = 0;
   GDMA.channel[dma_num].in.in_link.stop = 1;
   GDMA.channel[dma_num].in.in_link.addr = 0;

   if (gpio_isr_handle)
   {
      esp_intr_free(gpio_isr_handle);
      gpio_isr_handle = NULL;
#ifdef HI_LEVEL_INT_RISCV
      if(hi_level_ivect_idx)
      {_vector_table_to_write[hi_level_ivect_idx] = hi_level_backup_ivect_data;}
#endif      
   }
}
esp_err_t logic_analyzer_ll_init_dma_eof_isr(TaskHandle_t task)
{
   esp_err_t ret = esp_intr_alloc(gdma_periph_signals.groups[0].pairs[dma_num].rx_irq_id,
                                  ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
                                  la_ll_dma_isr, (void *)task, &isr_handle);
   if (ret != ESP_OK)
   {
      ESP_LOGE(TAG, "DMA interrupt allocation of analyzer failed error=%x", ret);
      return ret;
   }
   return ret;
}
void logic_analyzer_ll_deinit_dma_eof_isr()
{
   esp_intr_free(isr_handle);
}
