#pragma once


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include <driver/gpio.h>
#include "hal/gpio_types.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include "soc/i2s_struct.h"
#include "esp32/rom/lldesc.h"
#include "esp_private/periph_ctrl.h"

#define LA_CLK_SAMPLE_RATE 80000000
#define LA_MAX_SAMPLE_RATE 20000000
#define LA_MIN_SAMPLE_RATE 4000
#define LA_TASK_STACK 2048
#define DMA_FRAME 4092

/**
 * @brief Data structure of logic analizer frame buffer
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
 *  @brief logic analizer config i2s
 *        configure all i2s struct,before stert 
 *
 *  @param- int data_pins   - pointer of data GPIO array pin[16] ( 0-15 )
 *  @param- int pin_trigger - trigger GPIO ( -1 disable )
 *  @param- int sample_rate - real sample rate in HZ
 *  @param- la_frame_t *frame - pointer of dma frame ( dma desriptor, sample buffer, sample buffer len )
 *
 *  @return
*/
void logic_analizer_ll_config(int *data_pins,int pin_trigger,int sample_rate,la_frame_t *frame);
/**
 *  @brief logic analizer start meashure
 *
*/
void logic_analizer_ll_start();
/**
 *  @brief logic analizer stop meashure
 *
*/
void  logic_analizer_ll_stop();
/**
 *  @brief logic analizer init dma eof isr
 *          isr after full dma transfer
 *  @param-  TaskHandle_t task  - notify main task after full dma transfer
 *
 *  @return
*/
esp_err_t logic_analizer_ll_init_dma_eof_isr(TaskHandle_t task);
/**
 *  @brief logic analizer free dma eof isr
 *
 *  @return
*/
void logic_analizer_ll_deinit_dma_eof_isr();
/**
 *  @brief logic analizer return real sample rate 
 *
 *  @param  int sample_rate  - config sample rate
 *
 *  @return  real sample rate
*/
int logic_analizer_ll_get_sample_rate(int sample_rate);