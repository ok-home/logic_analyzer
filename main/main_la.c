#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "logic_analizer.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_4_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY (8)                  // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY (1250000)       // Frequency in Hertz. Set frequency at 5 kHz
#define LEDC_OUTPUT_IO (19)            // Define the output GPIO

#define GPIO_BLINK (15)
static TaskHandle_t tx_handle;
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

        vTaskDelay(1);
    }
}

void la_cb(uint16_t *buf, int cnt, int clk)
{
    if(buf==NULL) {printf("ERR timeout ??\n"); return;}
    printf("Data Ready CB\n");
    printf("cnt samples %d clock %d \n", cnt, clk);
    for (int i = 0; i < cnt + 16; i++)
    {
        if (i % 16 == 0)
        {
            printf("\n %04d ", i);
        }
        printf("%04x ", buf[i]);
    }
    printf("\nExit CB\n");
        vTaskDelete(tx_handle);
}
#define GPIO_TX 1
#define GPIO_RX 3
logic_analizer_config_t la_cfg =
    {
//        .pin = {LEDC_OUTPUT_IO, -1, -1, -1, GPIO_BLINK, -1, -1, -1,-1, -1, -1, -1, -1, -1, -1, -1},
        .pin = {-1, -1, -1, -1, -1, -1, -1, -1, GPIO_TX, -1, -1, -1, -1, -1, -1, -1},
        .pin_trigger = GPIO_TX,
        .trigger_edge = GPIO_INTR_ANYEDGE,
        .number_of_samples = 10000,
        .sample_rate = 1250000,
        .meashure_timeout = portMAX_DELAY,
        .logic_analizer_cb = la_cb
        };
    void txtask(void* p)
    {
        for(int i=0;;i++)
        {
            printf("data %02d\n",i);
        }
    }

void app_main(void)
{

    // Set the LEDC peripheral configuration
    //example_ledc_init();
    // Set duty to 50%
    //ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    //ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    //xTaskCreate(led_blink, "tt", 2048, NULL, 1, NULL);
    xTaskCreate(txtask, "tx", 2048, NULL, 1, &tx_handle);
    int ret = start_logic_analizer(&la_cfg);
    if (ret != ESP_OK)
        printf("ERR %x\n", ret);

};
