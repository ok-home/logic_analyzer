
#pragma once

#include "la.h"
#include "soc/i2s_struct.h"
#include "esp32/rom/lldesc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration structure for camera initialization
 */
typedef struct {
    //int pin_xclk;                   /*!< GPIO pin for camera XCLK line */
    //int pin_pclk;                   /*!< GPIO pin for camera PCLK line */

    int pin_d7;                     /*!< GPIO pin for camera D7 line */
    int pin_d6;                     /*!< GPIO pin for camera D6 line */
    int pin_d5;                     /*!< GPIO pin for camera D5 line */
    int pin_d4;                     /*!< GPIO pin for camera D4 line */
    int pin_d3;                     /*!< GPIO pin for camera D3 line */
    int pin_d2;                     /*!< GPIO pin for camera D2 line */
    int pin_d1;                     /*!< GPIO pin for camera D1 line */
    int pin_d0;                     /*!< GPIO pin for camera D0 line */
    //int pin_vsync;                  /*!< GPIO pin for camera VSYNC line */
    //int pin_href;                   /*!< GPIO pin for camera HREF line */

    //int xclk_freq_hz;               /*!< Frequency of XCLK signal, in Hz. EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode */
    //int pclk_freq_hz;               /*!< Frequency of PCLK signal, in Hz. EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode */

    //int frame_size;         /*!< Size of the output image: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA  */
} la_config_t;

/**
 * @brief Data structure of camera frame buffer
 */
typedef struct {
    uint8_t * buf;              /*!< Pointer to the pixel data */
    size_t len;                 /*!< Length of the buffer in bytes */
} la_fb_t;

typedef struct {
    la_fb_t fb;
    lldesc_t *dma;
} la_frame_t;

/**
 * @brief Uninitialize the lcd_cam module
 *
 * @param handle Provide handle pointer to release resources
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Uninitialize fail
 */
esp_err_t la_deinit(void);

/**
 * @brief Initialize the lcd_cam module
 *
 * @param config Configurations - see lcd_cam_config_t struct
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM No memory to initialize lcd_cam
 *     - ESP_FAIL Initialize fail
 */
esp_err_t la_init(la_config_t *config);

esp_err_t la_config(la_frame_t *config);

void la_stop(void);

void la_start(void);
#ifdef __cplusplus
}
#endif
