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

#include "logic_analyzer_pin_definition.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Data structure of logic analyzer frame buffer
 */
typedef struct {
    uint8_t * buf;              //Pointer to the sample data
    size_t len;                 //Length of the buffer in bytes
} la_fb_t;

typedef struct {
    la_fb_t fb;
    lldesc_t *dma;              //Pointer of dma frame
} la_frame_t;

/**
 *  @brief logic analyzer config i2s
 *        configure all i2s struct,before stert 
 *
 *  @param- int data_pins   - pointer of data GPIO array pin[16] ( 0-15 )
 *  @param- int pin_trigger - trigger GPIO ( -1 disable )
 *  @param- int sample_rate - real sample rate in HZ
 *  @param- la_frame_t *frame - pointer of dma frame ( dma desriptor, sample buffer, sample buffer len )
 *
 *  @return
*/
void logic_analyzer_ll_config(int *data_pins,int sample_rate,la_frame_t *frame);
/**
 *  @brief logic analyzer start meashure
 *
*/
void logic_analyzer_ll_start();

void logic_analyzer_ll_triggered_start(int pin_trigger,int trigger_edge);


/**
 *  @brief logic analyzer stop meashure
 *
*/
void  logic_analyzer_ll_stop();
/**
 *  @brief logic analyzer init dma eof isr
 *          isr after full dma transfer
 *  @param-  TaskHandle_t task  - notify main task after full dma transfer
 *
 *  @return
*/
esp_err_t logic_analyzer_ll_init_dma_eof_isr(TaskHandle_t task);
/**
 *  @brief logic analyzer free dma eof isr
 *
 *  @return
*/
void logic_analyzer_ll_deinit_dma_eof_isr();
/**
 *  @brief logic analyzer return real sample rate 
 *
 *  @param  int sample_rate  - config sample rate
 *
 *  @return  real sample rate
*/
int logic_analyzer_ll_get_sample_rate(int sample_rate);

// from hi-level nterrupt
void ll_hi_level_triggered_isr_start(int pin_trigger,int trigger_edge);
void ll_hi_level_triggered_isr_timeout_stop(void);


/*
* I2S0/I2S1 menuconfig select
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


#ifdef __cplusplus
}
#endif