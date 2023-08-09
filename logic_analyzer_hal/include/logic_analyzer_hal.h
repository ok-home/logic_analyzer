#pragma once

#include "stdint.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief logic analyzer callback
 *
 * @param- uint8_t *samle_buf   -  return pointer of samples
 * @param- int samples           - count of samples in 8/16 bit word depending of sample width
 * @param- int sample_rate       - real sample rate in HZ
 * if param == 0 -> timeout detected
 *
 */
typedef void (*logic_analyzer_cb_t)(uint8_t *samle_buf, int samples, int sample_rate); 

typedef struct {
    int pin[16];                // GPIO pin ESP32=>(0-39) ESP32S3=>(0-48) , -1 - disable , on 8bit mode use lower 8 pin [0,7]
    int pin_trigger;            // trigger GPIO pin, -1 - disable
    int trigger_edge;           // POS_EDGE/NEG_EDGE/ANY_EDGE
    int number_of_samples;      // Number of samples in 8/16 bit word depending of sample width
    int sample_rate;            // Sample rate in HZ ( 5000-80000000 ) depending of target, sample width, use psram
    int meashure_timeout;       // MAX meashured time in FreeRtos Tick - if timeout = 0 - stop&reset logic analyser
    logic_analyzer_cb_t logic_analyzer_cb ; // logic analyzer callback
} logic_analyzer_config_t;

/**
 * @brief Start logic analyzer
 *
 * @param config Configurations - see logic_analyzer_config_t struct
 *
 * @return
 *     - ESP_OK                 Success
 *     - ESP_ERR_INVALID_ARG    Parameter error
 *     - ESP_ERR_NO_MEM         No memory to initialize logic_analyzer
 *     - ESP_ERR_INVALID_STATE  logic_analyzer already working
 *     - ESP_FAIL               Initialize fail
 */
esp_err_t start_logic_analyzer(logic_analyzer_config_t *config);

#ifdef __cplusplus
}
#endif
