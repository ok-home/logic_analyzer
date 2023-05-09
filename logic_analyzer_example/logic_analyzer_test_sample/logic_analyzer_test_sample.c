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

#include "logic_analyzer_pin_definition.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_4_BIT // Set duty resolution to 
#define LEDC_DUTY (8)                  // Set duty to 50%. 
#define LEDC_FREQUENCY (500000)       // Frequency in Hertz. Set frequency at 500 kHz

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

void led_blink(void *p)
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
/*        
    uint32_t mask = 1<<GPIO_BLINK;
        
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
    */
    
    
    
    
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
    xTaskCreate(led_blink, "tt", 2048*2, NULL, 1, NULL);

}
