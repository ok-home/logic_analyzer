#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include <driver/gpio.h>
#include "hal/gpio_types.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "soc/i2s_struct.h"
#include "soc/gpio_reg.h"
//#include "soc/lldesc.h"
#include "esp_private/periph_ctrl.h"

#include "esp_idf_version.h"

#ifdef CONFIG_SPIRAM
#define LA_HW_PSRAM 1
#endif
// cache err -> define only > 5.2.0
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
#undef LA_HW_PSRAM
#endif

#define LA_HW_CLK_SAMPLE_RATE 160000000

#define LA_HW_MIN_GPIO -1
#define LA_HW_MAX_GPIO 54

#define LA_MIN_SAMPLE_RATE 200000



#define DMA_ALIGN 64
#define GDMA_BURST_CONST 3 // 0-8, 1-16, 2-32, 3-64, 4-128 

#define LA_HW_MAX_CHANNELS 16
#define LA_HW_MIN_CHANNELS 8

#define LA_HW_MIN_8_SAMPLE_RATE LA_MIN_SAMPLE_RATE
#define LA_HW_MIN_8_SAMPLE_CNT 100
#define LA_HW_MIN_16_SAMPLE_RATE LA_MIN_SAMPLE_RATE
#define LA_HW_MIN_16_SAMPLE_CNT 100

#define LA_HW_MAX_PSRAM_8_SAMPLE_RATE 80000000
#define LA_HW_MAX_PSRAM_16_SAMPLE_RATE 80000000
#define LA_HW_MAX_RAM_8_SAMPLE_RATE 80000000
#define LA_HW_MAX_RAM_16_SAMPLE_RATE 80000000

#define LA_HW_MAX_PSRAM_8_SAMPLE_CNT 32000000
#define LA_HW_MAX_PSRAM_16_SAMPLE_CNT 16000000
#define LA_HW_MAX_RAM_8_SAMPLE_CNT 400000
#define LA_HW_MAX_RAM_16_SAMPLE_CNT 200000
