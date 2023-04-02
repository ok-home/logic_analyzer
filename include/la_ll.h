
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

#include "la.h"
//#include "la_hal.h"
//#include "la_ll.h"




#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4092)



void la_ll_stop();
void la_ll_start(la_frame_t *frame);
void la_ll_config();
void la_ll_deinit();
void la_ll_set_pin(const la_config_t *config);
esp_err_t la_ll_init_isr();
