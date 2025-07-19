/* logic analyzer test sample example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"

// definition test sample
// esp32 uart -> 1-tx 3-rx
// esp32s3 uart -> 43-tx 44-rx
// esp32c3 uart -> 21-tx 20-rx
// output pin ledc example
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32P4) || defined(CONFIG_IDF_TARGET_ESP32S3)

#define LEDC_OUTPUT_IO (6)
// output pin gpio blink example
#define GPIO_BLINK (7)
// input wired example pins
#define IN_PORT_1 (14)
#define IN_PORT_2 (15)

#define GPIO_IRQ_PIN_26 (4)
#define GPIO_IRQ_PIN_27 (5)

#else

#define LEDC_OUTPUT_IO (12)
// output pin gpio blink example
#define GPIO_BLINK (13)
// input wired example pins
#define IN_PORT_1 (14)
#define IN_PORT_2 (15)

#define GPIO_IRQ_PIN_26 (2)
#define GPIO_IRQ_PIN_27 (5)

#endif

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_4_BIT // Set duty resolution to
#define LEDC_DUTY (4)                  // Set duty to 50%.
#define LEDC_FREQUENCY (100000)        // Frequency in Hertz. Set frequency at 100 kHz

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
portMUX_TYPE gpio_spinlock = portMUX_INITIALIZER_UNLOCKED;
uint32_t mask = 1 << GPIO_BLINK;

void IRAM_ATTR isr_26_handle(void *p)
{
    gpio_set_level(GPIO_IRQ_PIN_26, 0);
}
void IRAM_ATTR isr_27_handle(void *p)
{
    gpio_set_level(GPIO_IRQ_PIN_27, 1);
}

void irq_gpio_blink(void *p)
{
    gpio_reset_pin(GPIO_IRQ_PIN_26);
    gpio_reset_pin(GPIO_IRQ_PIN_27);
    gpio_set_direction(GPIO_IRQ_PIN_26, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(GPIO_IRQ_PIN_27, GPIO_MODE_INPUT_OUTPUT);

    gpio_set_level(GPIO_IRQ_PIN_26, 0);
    gpio_set_level(GPIO_IRQ_PIN_27, 1);

    gpio_install_isr_service(0); // default
    gpio_set_intr_type(GPIO_IRQ_PIN_26, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(GPIO_IRQ_PIN_27, GPIO_INTR_LOW_LEVEL);
    gpio_isr_handler_add(GPIO_IRQ_PIN_26, isr_26_handle, NULL);
    gpio_isr_handler_add(GPIO_IRQ_PIN_27, isr_27_handle, NULL);
    gpio_intr_enable(GPIO_IRQ_PIN_26);
    gpio_intr_enable(GPIO_IRQ_PIN_27);

    while (1)
    {
        vTaskDelay(1);
        gpio_set_level(GPIO_IRQ_PIN_26, 1);
        gpio_set_level(GPIO_IRQ_PIN_27, 0);
    }
}

void IRAM_ATTR on_off()
{
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    portENTER_CRITICAL(&gpio_spinlock);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
    gpio_set_level(GPIO_BLINK, 1);
    gpio_set_level(GPIO_BLINK, 0);
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32P4)
    GPIO.out_w1ts.val = mask;
    GPIO.out_w1tc.val = mask;
    GPIO.out_w1ts.val = mask;
    GPIO.out_w1tc.val = mask;
    GPIO.out_w1ts.val = mask;
    GPIO.out_w1tc.val = mask;
    GPIO.out_w1ts.val = mask;
    GPIO.out_w1tc.val = mask;
    GPIO.out_w1ts.val = mask;
    GPIO.out_w1tc.val = mask;
    GPIO.out_w1ts.val = mask;
    GPIO.out_w1tc.val = mask;
    GPIO.out_w1ts.val = mask;
    GPIO.out_w1tc.val = mask;
    GPIO.out_w1ts.val = mask;
    GPIO.out_w1tc.val = mask;
    GPIO.out_w1ts.val = mask;
    GPIO.out_w1tc.val = mask;
    GPIO.out_w1ts.val = mask;
    GPIO.out_w1tc.val = mask;
#else
    GPIO.out_w1ts = mask;
    GPIO.out_w1tc = mask;
    GPIO.out_w1ts = mask;
    GPIO.out_w1tc = mask;
    GPIO.out_w1ts = mask;
    GPIO.out_w1tc = mask;
    GPIO.out_w1ts = mask;
    GPIO.out_w1tc = mask;
    GPIO.out_w1ts = mask;
    GPIO.out_w1tc = mask;
    GPIO.out_w1ts = mask;
    GPIO.out_w1tc = mask;
    GPIO.out_w1ts = mask;
    GPIO.out_w1tc = mask;
    GPIO.out_w1ts = mask;
    GPIO.out_w1tc = mask;
    GPIO.out_w1ts = mask;
    GPIO.out_w1tc = mask;
    GPIO.out_w1ts = mask;
    GPIO.out_w1tc = mask;
#endif
    portEXIT_CRITICAL(&gpio_spinlock);
}
void gpio_blink(void *p)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << GPIO_BLINK,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_cfg);
    // gpio_set_direction(15,GPIO_MODE_INPUT_OUTPUT);
    while (1)
    {

        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        portENTER_CRITICAL(&gpio_spinlock);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);
        gpio_set_level(GPIO_BLINK, 1);
        gpio_set_level(GPIO_BLINK, 0);

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32P4)
        GPIO.out_w1ts.val = mask;
        GPIO.out_w1tc.val = mask;
        GPIO.out_w1ts.val = mask;
        GPIO.out_w1tc.val = mask;
        GPIO.out_w1ts.val = mask;
        GPIO.out_w1tc.val = mask;
        GPIO.out_w1ts.val = mask;
        GPIO.out_w1tc.val = mask;
        GPIO.out_w1ts.val = mask;
        GPIO.out_w1tc.val = mask;
        GPIO.out_w1ts.val = mask;
        GPIO.out_w1tc.val = mask;
        GPIO.out_w1ts.val = mask;
        GPIO.out_w1tc.val = mask;
        GPIO.out_w1ts.val = mask;
        GPIO.out_w1tc.val = mask;
        GPIO.out_w1ts.val = mask;
        GPIO.out_w1tc.val = mask;
        GPIO.out_w1ts.val = mask;
        GPIO.out_w1tc.val = mask;
#else
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
        GPIO.out_w1ts = mask;
        GPIO.out_w1tc = mask;
#endif

        portEXIT_CRITICAL(&gpio_spinlock);

        on_off();

        vTaskDelay(1);
    }
}
/*
#include "esp_etm.h"
#include "driver/gpio_etm.h"
 static esp_etm_channel_config_t etm_channel_config = {0};
 static esp_etm_channel_handle_t etm_channel_handle;
 static gpio_etm_event_config_t gpio_etm_event_config = {0};
 static esp_etm_event_handle_t gpio_etm_event_handle;
 static gpio_etm_task_config_t gpio_etm_task_config = {0};
 static  esp_etm_task_handle_t gpio_etm_task_handle;

void etm_start_stop(void*p){
    gpio_reset_pin(9);
    gpio_set_direction(9,GPIO_MODE_OUTPUT);
    gpio_set_level(9,0);
    esp_err_t etm_err_ret = 0;
    while(1){
    etm_err_ret = esp_etm_new_channel(&etm_channel_config, &etm_channel_handle);

    gpio_etm_event_config.edge = 2;
    etm_err_ret |= gpio_new_etm_event(&gpio_etm_event_config, &gpio_etm_event_handle);
    etm_err_ret |= gpio_etm_event_bind_gpio(gpio_etm_event_handle, 6);

    gpio_etm_task_config.action = GPIO_ETM_TASK_ACTION_SET;
    etm_err_ret |= gpio_new_etm_task(&gpio_etm_task_config,&gpio_etm_task_handle);
    etm_err_ret |= gpio_etm_task_add_gpio(gpio_etm_task_handle, 9);// pin9 ??

    //etm_err_ret |= esp_etm_channel_disable(etm_channel_handle);
    etm_err_ret |= esp_etm_channel_connect(etm_channel_handle, gpio_etm_event_handle, gpio_etm_task_handle);
    etm_err_ret |= esp_etm_channel_enable(etm_channel_handle);

    vTaskDelay(1);
    //  stop & remove ETM
      esp_etm_channel_disable(etm_channel_handle);
      gpio_etm_task_rm_gpio(gpio_etm_task_handle, 9);
      esp_etm_del_task(gpio_etm_task_handle);
      esp_etm_del_event(gpio_etm_event_handle);
      esp_etm_del_channel(etm_channel_handle);
      gpio_set_level(9,0);
    }
    

}
*/
#include "soc/hp_sys_clkrst_struct.h"
#include "soc/lp_clkrst_struct.h"
#include "soc/parl_io_struct.h"
#include "soc/parlio_periph.h"
#include "soc/gpio_sig_map.h"
#include "soc/axi_dma_struct.h"
#include "soc/gdma_periph.h"
#include "soc/axi_dma_reg.h"
#include "rom/cache.h"
#include "esp_cache.h" //esp_mm
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"

#define MAX(a,b) ((a) > (b) ? (a) : (b))


typedef struct dma_descriptor_align8_s dmadesc_t;
struct dma_descriptor_align8_s {
    struct {
        uint32_t size : 12;         /*!< Buffer size */
        uint32_t length : 12;       /*!< Number of valid bytes in the buffer */
        uint32_t reversed24_27 : 4; /*!< Reserved */
        uint32_t err_eof : 1;       /*!< Whether the received buffer contains error */
        uint32_t reserved29 : 1;    /*!< Reserved */
        uint32_t suc_eof : 1;       /*!< Whether the descriptor is the last one in the link */
        uint32_t owner : 1;         /*!< Who is allowed to access the buffer that this descriptor points to */
    } dw0;                          /*!< Descriptor Word 0 */
    void *buffer;                   /*!< Pointer to the buffer */
    dmadesc_t *next;  /*!< Pointer to the next descriptor (set to NULL if the descriptor is the last one, e.g. suc_eof=1) */
    uint32_t free;
}; //__attribute__((aligned(8)));
ESP_STATIC_ASSERT(sizeof(dmadesc_t) == 16, "dma_descriptor_align8_t should occupy 16 bytes in memory");


#include "esp_log.h"
#define gpio_matrix_in(a, b, c) esp_rom_gpio_connect_in_signal(a, b, c)
#define gpio_matrix_out(a, b, c, d) esp_rom_gpio_connect_out_signal(a, b, c, d)

uint8_t inbuf[512];


void IRAM_ATTR dma_isr(void *arg){

    typeof(AXI_DMA.in[0].intr.st) status = AXI_DMA.in[0].intr.st;
    uint32_t stat = status.val;
    AXI_DMA.in[0].intr.clr.val = ~0;
    AXI_DMA.in[0].intr.ena.val = 0;

    gpio_matrix_in(62, PARLIO_RX_DATA15_PAD_IN_IDX, false); // 1
//    gpio_set_level(14,0);

    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_en = 0;
    PARL_IO.rx_start_cfg.rx_start = 0;

    AXI_DMA.in[0].conf.in_link1.inlink_stop_chn = 1;
    ESP_EARLY_LOGE("dma irq", "irq status=%lx", stat);

}

void parlio_clk(void *p)
{
    gpio_reset_pin(15);
    gpio_set_direction(15,GPIO_MODE_OUTPUT);
    gpio_matrix_out(15, PARLIO_RX_CLK_PAD_OUT_IDX, false, false);

    gpio_reset_pin(14);
    gpio_set_direction(14,GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_in(14, PARLIO_RX_DATA15_PAD_IN_IDX, false);
    

    HP_SYS_CLKRST.ref_clk_ctrl2.reg_ref_160m_clk_en = 1;
    HP_SYS_CLKRST.soc_clk_ctrl1.reg_parlio_sys_clk_en = 1;
    HP_SYS_CLKRST.soc_clk_ctrl2.reg_parlio_apb_clk_en = 1;

    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_src_sel = 2; // 2-PLL160
    //rx
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl117, reg_parlio_rx_clk_div_num,16-1); // 160/8
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl118, reg_parlio_rx_clk_div_denominator,0);
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl118, reg_parlio_rx_clk_div_numerator,0);

    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio = 1;
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio = 0;
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio_rx = 1;
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio_rx = 0;

    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_en = 0;

    PARL_IO.clk.clk_en = 0 ; // ??
    PARL_IO.rx_genrl_cfg.rx_gating_en = 0;

    esp_intr_alloc(gdma_periph_signals.groups[1].pairs[0].rx_irq_id,
                         ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
                         dma_isr, NULL, NULL);


    PARL_IO.int_clr.val = ~0;
    PARL_IO.int_ena.val = 0;

//    PARL_IO.rx_mode_cfg.rx_sw_en = 1; // 1-soft start mode
//    PARL_IO.rx_mode_cfg.rx_smp_mode_sel = 2; //2 soft

    PARL_IO.rx_mode_cfg.rx_sw_en = 0; // 1-soft start mode
    PARL_IO.rx_mode_cfg.rx_smp_mode_sel = 0; //0 LVL
    PARL_IO.rx_mode_cfg.rx_ext_en_sel = 0xf;


    PARL_IO.rx_data_cfg.rx_bus_wid_sel = 3;//8
    PARL_IO.rx_genrl_cfg.rx_eof_gen_sel = 1; //bit lenght eof

    HP_SYS_CLKRST.soc_clk_ctrl1.reg_ahb_pdma_sys_clk_en = 1;
    HP_SYS_CLKRST.soc_clk_ctrl1.reg_axi_pdma_sys_clk_en = 1;
    HP_SYS_CLKRST.hp_rst_en1.reg_rst_en_axi_pdma = 1;
    HP_SYS_CLKRST.hp_rst_en1.reg_rst_en_axi_pdma = 0;

    uint32_t cache_line_size = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_INT_MEM, CACHE_TYPE_DATA);
    size_t alignment = MAX(cache_line_size, 32);
    uint32_t cache_line_cnt = ((1 + 1) * sizeof(dmadesc_t) + alignment)/alignment;
    dmadesc_t *dma_p = (dmadesc_t *)heap_caps_aligned_calloc(alignment,cache_line_cnt ,alignment, MALLOC_CAP_DMA);

        dma_p[0].dw0.size = 256;
        dma_p[0].dw0.length = 256;
        dma_p[0].dw0.suc_eof = 0;
        dma_p[0].dw0.owner = 1;
        dma_p[0].buffer = inbuf;
        dma_p[0].next = NULL;
  
        esp_cache_msync ( (void*)dma_p , cache_line_size , ESP_CACHE_MSYNC_FLAG_DIR_C2M ) ;

    gpio_matrix_in(62, PARLIO_RX_DATA15_PAD_IN_IDX, false); // 1

    while(1)
    {
        vTaskDelay(4);
        HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_en = 0;

        PARL_IO.rx_start_cfg.rx_start = 0;
        AXI_DMA.in[0].conf.in_link1.inlink_stop_chn = 1;

        esp_cache_msync ( (void*)dma_p , cache_line_size , ESP_CACHE_MSYNC_FLAG_DIR_M2C ) ;

        ESP_LOGI("parlio", "len=%d size=%d err=%d eof=%d own=%d ptr=%p",dma_p[0].dw0.length,dma_p[0].dw0.size,
                    dma_p[0].dw0.err_eof,dma_p[0].dw0.suc_eof,dma_p[0].dw0.owner, dma_p[0].buffer);
        dma_p[0].dw0.owner = 1;
        esp_cache_msync ( (void*)dma_p , cache_line_size , ESP_CACHE_MSYNC_FLAG_DIR_C2M ) ;


        AXI_DMA.in[0].intr.clr.val = ~0;
        AXI_DMA.in[0].intr.ena.val = 0;

        AXI_DMA.in[0].conf.in_conf1.in_check_owner_chn = 1;
        AXI_DMA.in[0].conf.in_conf0.in_rst_chn = 1;
        AXI_DMA.in[0].conf.in_conf0.in_rst_chn = 0;
        AXI_DMA.in[0].conf.in_link2.inlink_addr_chn = (uint32_t)dma_p; // set dma descriptor
        AXI_DMA.in[0].conf.in_peri_sel.peri_in_sel_chn = 3; // 0:lcdcam. 1: gpspi_2. 2: gpspi_3. 3: parl_io. 4: aes. 5: sha. 6~15: Dummy

        PARL_IO.fifo_cfg.rx_fifo_srst = 1; // reset fifo
        PARL_IO.fifo_cfg.rx_fifo_srst = 0; // reset fifo

        PARL_IO.rx_data_cfg.rx_bitlen = 8; // any on ext level mode
        PARL_IO.reg_update.rx_reg_update = 1;

        AXI_DMA.in[0].intr.ena.val = ~0;
        AXI_DMA.in[0].intr.clr.val = ~0;


        AXI_DMA.in[0].conf.in_link1.inlink_start_chn = 1;
        PARL_IO.rx_start_cfg.rx_start = 1;
        HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_en = 1;

        gpio_matrix_in(63, PARLIO_RX_DATA15_PAD_IN_IDX, false); // 1

//        gpio_set_level(14,1);

    }

}

void test_sample_init(void)
{
    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    xTaskCreate(gpio_blink, "tt", 2048 * 2, NULL, 1, NULL);
    xTaskCreate(irq_gpio_blink, "irq", 2048 * 2, NULL, 1, NULL);
    //xTaskCreate(etm_start_stop, "etm", 2048 * 2, NULL, 1, NULL);
    //xTaskCreate(parlio_clk,"parlio",2048*2,NULL,1,NULL);

}
