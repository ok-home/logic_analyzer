
#pragma once

#include "soc/i2s_struct.h"
#include "esp32/rom/lldesc.h"

#ifdef __cplusplus
extern "C" {
#endif

/
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

#ifdef __cplusplus
}
#endif
