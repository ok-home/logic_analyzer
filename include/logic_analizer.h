#pragma once

#include "esp_err.h"
#include "sys/time.h"
#include "sdkconfig.h"

#include "la_hal.h"
#include "la_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LA_PIN_D7 15
#define LA_PIN_D6 16
#define LA_PIN_D5 17
#define LA_PIN_D4 18
#define LA_PIN_D3 19
#define LA_PIN_D2 21
#define LA_PIN_D1 22
#define LA_PIN_D0 23

#define LA_PIN_TRIGGER 25


/**
 * @brief logic analizer callback
 *
 * @param- uint16_t *samle_buf   - return pointer of samples
 * @param- int samples           - count of samples in 16 bit word
 * @param- int sample_rate       - real sample rate in HZ
 *
 * @return
*/
typedef void (logic_analizer_cb_t *)(uint16_t *samle_buf, int samples, int sample_rate); 

typedef struct {
    int pin[16];                // GPIO pin (0-39), -1 - disable
    int pin_trigger;            // trigger GPIO pin, -1 - disable
    int trigger_edge;           // POS_EDGE/NEG_EDGE/ANY_EDGE
    int number_of_samples;      // Number of samples in 16 bit word
    int sample_rate;            // Sample rate in HZ ( 4 000-20 000 000 )
    int meashure_timeout;       // MAX meashured time in FreeRtos Tick - call cb function with zero buff&samples on timeout
    logic_analizer_cb_t logic_analizer_cb ; // logic analizer callback
} logic_analizer_config_t;
/**
 * @brief Start logic analizer
 *
 * @param config Configurations - see logic_analizer_config_t struct
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM No memory to initialize logic_analizer
 *     - ?????????????? - logic_analizer already working
 *     - ESP_FAIL Initialize fail
 */
esp_err_t start_logic_analizer(logic_analizer_config_t *config);

// https://godbolt.org/z/Y39TrhzPT

#ifdef __cplusplus
}
#endif
