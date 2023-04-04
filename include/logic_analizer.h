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
 * @brief Uninitialize the lcd_cam module
 *
 * @param handle Provide handle pointer to release resources
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Uninitialize fail
 */

typedef struct {
    int pin[16];                // GPIO pin (0-39), -1 - disable
    int pin_trigger;            // trigger GPIO pin, -1 - disable
    int trigger_edge;           // POS_EDGE/NEG_EDGE/ANY_EDGE
    int number_of_samples;      // Number of samples in 16 bit word
    int sample_rate;            // Sample rate in HZ ( 1-20000000 )
    int meashure_timeout;       // MAX meashured time in FreeRtos Tick - call cb function with zero buff&samples on timeout
    void (logic_analizer_cb *)(uint16_t *samle_buf,uint32_t *samples); // logic analizer callback, return pointer of samples & count of samples in 16 bit word
} logic_analizer_config_t;

esp_err_t start_logic_analizer(ogic_analizer_config_t *config);
int get_logic_analizer_sample_rate();





#ifdef __cplusplus
}
#endif
