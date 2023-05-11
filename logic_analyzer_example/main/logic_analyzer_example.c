
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

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
    // test uart out
    while(1)
    {
        //ESP_LOGI("TEST MSG","TEST 123456789");
        vTaskDelay(100);
    }
}