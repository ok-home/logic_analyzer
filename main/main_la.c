#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "logic_analizer_hal.h"
#include "logic_analizer_sump.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

extern void test_sample_init();
extern void test_air();
void app_main(void)
{
    test_sample_init();
    test_air();
    vTaskDelay(5);
    xTaskCreate(sump_task, "sump_task", 2048*4, NULL, 1, NULL);
}
