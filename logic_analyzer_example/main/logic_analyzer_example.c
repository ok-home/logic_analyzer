
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
    // extern void test_air();
    test_sample_init();
    // test_air();
    vTaskDelay(5);
#ifdef CONFIG_ANALYZER_USE_SUMP
#ifdef CONFIG_IDF_TARGET_ESP32
    esp_log_level_set("*", ESP_LOG_NONE); // sump default run on port0 - disable log
#endif
    logic_analyzer_sump();
#endif
#ifdef CONFIG_ANALYZER_USE_WS
    logic_analyzer_ws_server();
#endif
    while (1)
    {
        vTaskDelay(100);
    }
}