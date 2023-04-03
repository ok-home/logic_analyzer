#include <stdio.h>
#include "la.h"
// #include "la_hal.h"
// #include "la_ll.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (19) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_4_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (8) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (1250000) // Frequency in Hertz. Set frequency at 5 kHz

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

gpio_config_t io_cfg = {
    .pin_bit_mask = 1ULL<<15,
    .mode = GPIO_MODE_OUTPUT,
    .intr_type=GPIO_INTR_DISABLE
};

void led_blink(void *p)
{
    gpio_config(&io_cfg);
    //gpio_set_direction(15,GPIO_MODE_INPUT_OUTPUT);
    while (1)
    {
        gpio_set_level(15,1);
        vTaskDelay(1);
        gpio_set_level(15,0);
        vTaskDelay(1);
    }
}

void app_main(void)
{
    printf("hello\n");
// Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    xTaskCreate(led_blink, "tt", 2048, NULL, 1, NULL);
    la_start();
};
/*
200 СЛОВ 10 МСЕК
20 1 МСЕК - 20 КГЦ

5330-6113-6897
783      784
12,5МКС
*/
