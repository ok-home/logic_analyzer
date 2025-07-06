#pragma once

#include "logic_analyzer_const_definition.h"
#include "logic_analyzer_pin_definition.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct dma_descriptor_align8_s lldesc_t;
struct dma_descriptor_align8_s {
    struct {
        uint32_t size : 12;         /*!< Buffer size */
        uint32_t length : 12;       /*!< Number of valid bytes in the buffer */
        uint32_t reversed24_27 : 4; /*!< Reserved */
        uint32_t err_eof : 1;       /*!< Whether the received buffer contains error */
        uint32_t reserved29 : 1;    /*!< Reserved */
        uint32_t suc_eof : 1;       /*!< Whether the descriptor is the last one in the link */
        uint32_t owner : 1;         /*!< Who is allowed to access the buffer that this descriptor points to */
    } dw0;                          /*!< Descriptor Word 0 */
    void *buffer;                   /*!< Pointer to the buffer */
    lldesc_t *next;  /*!< Pointer to the next descriptor (set to NULL if the descriptor is the last one, e.g. suc_eof=1) */
    uint32_t free;
}; //__attribute__((aligned(8)));
ESP_STATIC_ASSERT(sizeof(lldesc_t) == 16, "dma_descriptor_align8_t should occupy 16 bytes in memory");



    /**
     * @brief Data structure of logic analyzer frame buffer
     */
    typedef struct
    {
        uint8_t *buf; // Pointer to the sample data
        size_t len;   // Length of the buffer in bytes
    } la_fb_t;

    typedef struct
    {
        la_fb_t fb;
        lldesc_t *dma; // Pointer of dma frame
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
    void logic_analyzer_ll_config(int *data_pins, int sample_rate, int channels, la_frame_t *frame);
    /**
     *  @brief logic analyzer start meashure
     *
     */
    void logic_analyzer_ll_start();

    void logic_analyzer_ll_triggered_start(int pin_trigger, int trigger_edge);

    /**
     *  @brief logic analyzer stop meashure
     *
     */
    void logic_analyzer_ll_stop();
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
    void ll_hi_level_triggered_isr_start(int pin_trigger, int trigger_edge);
    void ll_hi_level_triggered_isr_timeout_stop(void);

#ifdef __cplusplus
}
#endif