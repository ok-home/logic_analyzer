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


#define LA_CLK_SAMPLE_RATE 80000000
#define LA_MAX_SAMPLE_RATE 40000000
#define LA_MIN_SAMPLE_RATE 5000

#define LA_MAX_PIN 16
#define LA_MIN_GPIO -1
#define LA_MAX_GPIO 39
#define LA_MAX_SAMPLE_CNT 60000
#define LA_MIN_SAMPLE_CNT 100
#define LA_BYTE_IN_SAMPLE 2

/*
* I2S0/I2S1 menuconfig select - only esp32 target
*/

#ifdef CONFIG_ANALYZER_USE_I2S_CHANNEL_0

#define PERIPH_I2SX_MODULE PERIPH_I2S0_MODULE
#define I2SX I2S0
#define I2SXI_V_SYNC_IDX I2S0I_V_SYNC_IDX
#define I2SXI_H_SYNC_IDX I2S0I_H_SYNC_IDX
#define I2SXI_H_ENABLE_IDX I2S0I_H_ENABLE_IDX
#define I2SXI_DATA_IN0_IDX I2S0I_DATA_IN0_IDX
#define ETS_I2SX_INTR_SOURCE ETS_I2S0_INTR_SOURCE
#define GPIO_FUNC_V_SYNC_IN_SEL_CFG_REG GPIO_FUNC191_IN_SEL_CFG_REG

#elif CONFIG_ANALYZER_USE_I2S_CHANNEL_1

#define PERIPH_I2SX_MODULE PERIPH_I2S1_MODULE
#define I2SX I2S1
#define I2SXI_V_SYNC_IDX I2S1I_V_SYNC_IDX
#define I2SXI_H_SYNC_IDX I2S1I_H_SYNC_IDX
#define I2SXI_H_ENABLE_IDX I2S1I_H_ENABLE_IDX
#define I2SXI_DATA_IN0_IDX I2S1I_DATA_IN0_IDX
#define ETS_I2SX_INTR_SOURCE ETS_I2S1_INTR_SOURCE
#define GPIO_FUNC_V_SYNC_IN_SEL_CFG_REG GPIO_FUNC194_IN_SEL_CFG_REG

#endif
