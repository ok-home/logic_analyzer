#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include <driver/gpio.h>
#include "hal/gpio_types.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "soc/i2s_struct.h"
#include "soc/dport_reg.h"
#include "soc/gpio_reg.h"
#include "esp32/rom/lldesc.h"
#include "esp_private/periph_ctrl.h"

#include "esp_idf_version.h"

#undef LA_HW_PSRAM


#undef LA_HW_PSRAM

#define LA_HW_CLK_SAMPLE_RATE 160000000

#define LA_HW_MIN_GPIO -1
#define LA_HW_MAX_GPIO 21
/*
#ifdef CONFIG_ANALYZER_USE_LEDC_TIMER_FOR_PCLK
#define LA_MIN_SAMPLE_RATE 20000
#else
#define LA_MIN_SAMPLE_RATE 1000000
#endif
*/

#define LA_HW_CLK_SAMPLE_RATE 160000000
#define LA_HW_MIN_GPIO -1
#define LA_HW_MAX_GPIO 21

#define LA_MAX_SAMPLE_RATE 80000000
#define LA_MIN_SAMPLE_RATE 20000
#define LA_MAX_SAMPLE_CNT 60000
#define LA_MIN_SAMPLE_CNT 100

#define LA_HW_MAX_CHANNELS 8
#define LA_HW_MIN_CHANNELS 8

#define LA_HW_MIN_8_SAMPLE_RATE LA_MIN_SAMPLE_RATE
#define LA_HW_MIN_8_SAMPLE_CNT LA_MIN_SAMPLE_CNT
#define LA_HW_MIN_16_SAMPLE_RATE LA_MIN_SAMPLE_RATE
#define LA_HW_MIN_16_SAMPLE_CNT LA_MIN_SAMPLE_CNT

#define LA_HW_MAX_PSRAM_8_SAMPLE_RATE LA_MAX_SAMPLE_RATE
#define LA_HW_MAX_PSRAM_16_SAMPLE_RATE LA_MAX_SAMPLE_RATE
#define LA_HW_MAX_RAM_8_SAMPLE_RATE LA_MAX_SAMPLE_RATE
#define LA_HW_MAX_RAM_16_SAMPLE_RATE LA_MAX_SAMPLE_RATE

#define LA_HW_MAX_PSRAM_8_SAMPLE_CNT LA_MAX_SAMPLE_CNT
#define LA_HW_MAX_PSRAM_16_SAMPLE_CNT LA_MAX_SAMPLE_CNT
#define LA_HW_MAX_RAM_8_SAMPLE_CNT LA_MAX_SAMPLE_CNT
#define LA_HW_MAX_RAM_16_SAMPLE_CNT LA_MAX_SAMPLE_CNT


