/* logic analyzer test example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "logic_analyzer_hal.h"
#include "logic_analyzer_sump.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

extern void test_sample_init();
extern void test_air();
void app_main(void)
{
    test_sample_init();
    test_air();
    vTaskDelay(5);
    xTaskCreate(logic_analyzer_sump_task, "sump_task", 2048*4, NULL, 1, NULL);
}
