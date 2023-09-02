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
// 43-tx 44-rx
// output pin ledc example
#ifdef CONFIG_IDF_TARGET_ESP32C3
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
#ifdef CONFIG_IDF_TARGET_ESP32C3
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

#ifdef CONFIG_IDF_TARGET_ESP32C3
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
void test_sample_init(void)
{
    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    xTaskCreate(gpio_blink, "tt", 2048 * 2, NULL, 1, NULL);
    xTaskCreatePinnedToCore(irq_gpio_blink, "irq", 2048 * 2, NULL, 1, NULL, 1);
}
