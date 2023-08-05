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


#define LA_CLK_SAMPLE_RATE 160000000
#define LA_MIN_SAMPLE_RATE 1000000
#define LA_MIN_SAMPLE_CNT 100
#define LA_MIN_GPIO -1
#define LA_MAX_GPIO 48

#ifdef CONFIG_ANALYZER_CHANNEL_NUMBERS_8
#define LA_MAX_SAMPLE_CNT 64000
#define LA_MAX_SAMPLE_RATE 80000000
#define LA_MAX_PIN 8
#define LA_BYTE_IN_SAMPLE 1
#else
#define LA_MAX_SAMPLE_CNT 32000
#define LA_MAX_SAMPLE_RATE 40000000
#define LA_MAX_PIN 16
#define LA_BYTE_IN_SAMPLE 2
#endif


