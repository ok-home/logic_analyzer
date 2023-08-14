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

// cache err -> define only > 5.2.0
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
#undef CONFIG_ANALYZER_USE_PSRAM
#endif


#define LA_CLK_SAMPLE_RATE 160000000

#define LA_MIN_SAMPLE_CNT 100
#define LA_MIN_GPIO -1
#define LA_MAX_GPIO 48

#ifdef CONFIG_ANALYZER_USE_LEDC_TIMER_FOR_PCLK
#define LA_MIN_SAMPLE_RATE 20000
#else
#define LA_MIN_SAMPLE_RATE 1000000
#endif

#ifdef CONFIG_ANALYZER_CHANNEL_NUMBERS_8
 #ifdef CONFIG_ANALYZER_USE_PSRAM
  #define LA_MAX_SAMPLE_CNT 8000000
  #define LA_MAX_SAMPLE_RATE 20000000
 #else
  #define LA_MAX_SAMPLE_CNT 200000
  #define LA_MAX_SAMPLE_RATE 80000000
 #endif
#define LA_MAX_PIN 8
#define LA_MIN_PIN 8
#define LA_BYTE_IN_SAMPLE 1
#else
 #ifdef CONFIG_ANALYZER_USE_PSRAM
  #define LA_MAX_SAMPLE_CNT 4000000
  #define LA_MAX_SAMPLE_RATE 10000000
 #else
  #define LA_MAX_SAMPLE_CNT 100000
  #define LA_MAX_SAMPLE_RATE 40000000
 #endif
#define LA_MAX_PIN 16
#define LA_MIN_PIN 16
#define LA_BYTE_IN_SAMPLE 2
#endif

#define LA_MIN_PSRAM 0
#ifdef CONFIG_ANALYZER_USE_PSRAM
//16-32
#define CONFIG_ANALYZER_GDMA_PSRAM_BURST 32
#define LA_MAX_PSRAM 1
#else
#define LA_MAX_PSRAM 0
#endif

#define LA_HW_MAX_CHANNELS 16
#define LA_HW_MIN_CHANNELS 8

#define LA_HW_MIN_SAMPLE_RATE 20000
#define LA_HW_MIN_SAMPLE_CNT 100

#define LA_HW_MAX_PSRAM_8_SAMPLE_RATE 20000000
#define LA_HW_MAX_PSRAM_16_SAMPLE_RATE 10000000
#define LA_HW_MAX_RAM_8_SAMPLE_RATE 20000000
#define LA_HW_MAX_RAM_16_SAMPLE_RATE 10000000

#define LA_HW_MAX_PSRAM_8_SAMPLE_CNT 8000000
#define LA_HW_MAX_PSRAM_16_SAMPLE_CNT 4000000
#define LA_HW_MAX_RAM_8_SAMPLE_CNT 200000
#define LA_HW_MAX_RAM_16_SAMPLE_CNT 100000

#define LA_HW_PSRAM 1
