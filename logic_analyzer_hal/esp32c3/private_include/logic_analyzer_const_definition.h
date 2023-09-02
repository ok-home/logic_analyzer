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
#include "esp32/rom/lldesc.h"
#include "esp_private/periph_ctrl.h"

#include "driver/spi_master.h"

#include "esp_idf_version.h"

#ifdef CONFIG_ANALYZER_USE_HI_LEVEL_INTERRUPT
#ifndef CONFIG_ESP_SYSTEM_MEMPROT_FEATURE
#define HI_LEVEL_INT_RISCV
#endif
#endif

#undef LA_HW_PSRAM
#undef CONFIG_ANALYZER_USE_HI_LEVEL_INTERRUPT

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

// ????
#define GDMA_PSRAM_BURST 32

#define LA_HW_CLK_SAMPLE_RATE 160000000
#define LA_HW_MIN_GPIO -1
#define LA_HW_MAX_GPIO 21

#define LA_MAX_SAMPLE_RATE 80000000
#define LA_MIN_SAMPLE_RATE 5000
#define LA_MAX_SAMPLE_CNT 64000
#define LA_MIN_SAMPLE_CNT 100

#define LA_HW_MAX_CHANNELS 4
#define LA_HW_MIN_CHANNELS 4

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
