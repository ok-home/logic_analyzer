
#pragma once

#include <stdint.h>
#include "sdkconfig.h"
#include "esp_idf_version.h"
#include "esp32/rom/lldesc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#if __has_include("esp_private/periph_ctrl.h")
# include "esp_private/periph_ctrl.h"
#endif

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
void IRAM_ATTR logic_analizer_ll_stop();
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