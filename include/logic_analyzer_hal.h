#pragma once

#include "stdint.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief logic analyzer callback
 *
 * @param- uint16_t *samle_buf   - return pointer of samples
 * @param- int samples           - count of samples in 16 bit word
 * @param- int sample_rate       - real sample rate in HZ
 *
 * @return
*/
typedef void (*logic_analyzer_cb_t)(uint16_t *samle_buf, int samples, int sample_rate); 

typedef struct {
    int pin[16];                // GPIO pin (0-39), -1 - disable
    int pin_trigger;            // trigger GPIO pin, -1 - disable
    int trigger_edge;           // POS_EDGE/NEG_EDGE/ANY_EDGE
    int number_of_samples;      // Number of samples in 16 bit word
    int sample_rate;            // Sample rate in HZ ( 4 000-20 000 000 )
    int meashure_timeout;       // MAX meashured time in FreeRtos Tick - call cb function with zero buff&samples on timeout
    logic_analyzer_cb_t logic_analyzer_cb ; // logic analyzer callback
} logic_analyzer_config_t;
/**
 * @brief Start logic analyzer
 *
 * @param config Configurations - see logic_analyzer_config_t struct
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM No memory to initialize logic_analyzer
 *     - 9 - logic_analyzer already working
 *     - ESP_FAIL Initialize fail
 */
esp_err_t start_logic_analyzer(logic_analyzer_config_t *config);

#ifdef __cplusplus
}
#endif
