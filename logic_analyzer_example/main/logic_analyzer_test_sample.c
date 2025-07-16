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
#include "esp_log.h"
#define gpio_matrix_in(a, b, c) esp_rom_gpio_connect_in_signal(a, b, c)
#define gpio_matrix_out(a, b, c, d) esp_rom_gpio_connect_out_signal(a, b, c, d)
void parlio_clk(void *p)
{
    gpio_reset_pin(15);
    gpio_set_direction(15,GPIO_MODE_OUTPUT);
    gpio_matrix_out(15, PARLIO_RX_CLK_PAD_OUT_IDX, false, false);

    HP_SYS_CLKRST.ref_clk_ctrl2.reg_ref_160m_clk_en = 1;
    
    HP_SYS_CLKRST.soc_clk_ctrl1.reg_parlio_sys_clk_en = 1;
    HP_SYS_CLKRST.soc_clk_ctrl2.reg_parlio_apb_clk_en = 1;

    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio = 1;
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio = 0;

    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio_rx = 1;
    HP_SYS_CLKRST.hp_rst_en2.reg_rst_en_parlio_rx = 0;

    
    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_src_sel = 2; // 2-PLL160

    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl117, reg_parlio_rx_clk_d); // 160/8
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl118, reg_parlio_rx_clk_div_denominator,0);
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl118, reg_parlio_rx_clk_div_numerator,0);

    LP_AON_CLKRST.hp_clk_ctrl.hp_pad_parlio_rx_clk_en = 1;
    HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_en = 1;


    PARL_IO.clk.clk_en = 1 ; // ??
    PARL_IO.rx_genrl_cfg.rx_gating_en = 0;


    ESP_LOGI("parlio", "GATE %d clk %d",PARL_IO.rx_genrl_cfg.rx_gating_en,PARL_IO.rx_genrl_cfg.rx_gating_en);

    while(1)
    {
        vTaskDelay(1);
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
    xTaskCreate(parlio_clk,"parlio",2048*2,NULL,1,NULL);

}
