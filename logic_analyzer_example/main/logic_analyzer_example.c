
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#ifdef CONFIG_ANALYZER_USE_SUMP
#include "logic_analyzer_sump.h"
#endif
#ifdef CONFIG_ANALYZER_USE_WS
#include "logic_analyzer_ws_server.h"
#endif

void app_main(void)
{
    extern void test_sample_init();
    //extern void test_air();
    test_sample_init();
    //test_air();
    vTaskDelay(5);
    #ifdef CONFIG_ANALYZER_USE_SUMP
    esp_log_level_set("*", ESP_LOG_NONE); // sump default run on port0 - disable log
    logic_analyzer_sump();
    #endif
    #ifdef CONFIG_ANALYZER_USE_WS
    logic_analyzer_ws_server();
    #endif

#define IN_DONE_DBG 6
#define IN_DSCR_EMPTY_DBG 7
#define IN_EOF_DBG 8 

    gpio_reset_pin(IN_DONE_DBG);
    gpio_set_direction(IN_DONE_DBG,GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN_DSCR_EMPTY_DBG);
    gpio_set_direction(IN_DSCR_EMPTY_DBG,GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN_EOF_DBG);
    gpio_set_direction(IN_EOF_DBG,GPIO_MODE_OUTPUT);
    gpio_set_level(IN_DONE_DBG,0);
    gpio_set_level(IN_DSCR_EMPTY_DBG,0);
    gpio_set_level(IN_EOF_DBG,0);

    while(1)
    {
        vTaskDelay(100);
    }
}