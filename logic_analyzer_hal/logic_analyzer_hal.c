/* logic analyzer hal example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "logic_analyzer_hal.h"
#include "logic_analyzer_ll.h"
#include "logic_analyzer_hi_level_interrupt.h"
#include "soc/soc.h"
#include "soc/dport_access.h"
#include "esp_log.h"
#include "string.h"
#include "rom/cache.h"

#define LA_TASK_STACK 2048
#define DMA_FRAME 4032

// frame buff & dma descripter
static la_frame_t la_frame = {
    .fb.buf = NULL,
    .fb.len = 0, // bytes *** one sample = 2 byte
    .dma = NULL};

static TaskHandle_t logic_analyzer_task_handle = 0; // main task handle
static int logic_analyzer_started = 0;              // flag start dma
//
// esp32 only
// sample sequence in 32 word - adr0=sample1, adr1=sample0
// swap sample sequence
//
#ifdef CONFIG_IDF_TARGET_ESP32
static inline void swap_buf(uint16_t *buf, int cnt)
{
    uint16_t tmp;
    for (int i = 0; i < cnt; i += 2)
    {
        tmp = buf[i];
        buf[i] = buf[i + 1];
        buf[i + 1] = tmp;
    }
}
#endif

/**
 * @brief allocate dma descriptor
 *
 * @param uint32_t size - size of sample frame buffer (bytes)
 * @param uint8_t *buffer - pointer of sample frame buffer
 *
 * @return
 *     - dma descriptor ( NULL if no mem )
 */
static lldesc_t *allocate_dma_descriptors(uint32_t size, uint8_t *buffer)
{
    uint32_t count = size / DMA_FRAME;     //  dma frames count
    uint32_t last_size = size % DMA_FRAME; // last frame bytes

    lldesc_t *dma = (lldesc_t *)heap_caps_malloc((count + 1) * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma == NULL)
    {
        return dma;
    }
    int x = 0;
    for (; x < count; x++)
    {
        dma[x].size = DMA_FRAME;
        dma[x].length = DMA_FRAME;
        dma[x].sosf = 0;
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = buffer + DMA_FRAME * x;
        dma[x].empty = (uint32_t)&dma[(x + 1)];
    }

    dma[x].size = last_size;
    dma[x].length = last_size;
    dma[x].sosf = 0;
    dma[x].eof = 1;
    dma[x].owner = 1;
    dma[x].buf = buffer + DMA_FRAME * x;
    dma[x].empty = 0;

    return dma;
}
/**
 *  @brief  full stop & free all
 */
static void logic_analyzer_stop(void)
{
    // stop dma transfer
    logic_analyzer_ll_stop();
    // deinit dma isr
    logic_analyzer_ll_deinit_dma_eof_isr();
    if (la_frame.dma)
    {
        free(la_frame.dma);
        la_frame.dma = NULL;
    }
    if (la_frame.fb.buf)
    {
        free(la_frame.fb.buf);
        la_frame.fb.buf = NULL;
        la_frame.fb.len = 0;
    }
    logic_analyzer_started = 0;
}
/**
 *  @brief main logic analyzer task
 *       call callback function aftr dma transfer
 *       callback param = 0 if timeout
 */
static void logic_analyzer_task(void *arg)
{
    int noTimeout;
    logic_analyzer_config_t *cfg = (logic_analyzer_config_t *)arg;

    while (1)
    {
        noTimeout = ulTaskNotifyTake(pdFALSE, cfg->meashure_timeout); // portMAX_DELAY
        if (noTimeout)
        {
            // dma data ready
            // esp32 -> sample sequence in 32 word - adr0=sample1, adr1=sample0
            // swap sample sequence on esp32.
#ifdef CONFIG_IDF_TARGET_ESP32
            swap_buf((uint16_t *)la_frame.fb.buf, la_frame.fb.len / LA_BYTE_IN_SAMPLE);
#endif
            cfg->logic_analyzer_cb((uint8_t *)la_frame.fb.buf, la_frame.fb.len / LA_BYTE_IN_SAMPLE, logic_analyzer_ll_get_sample_rate(cfg->sample_rate));
            logic_analyzer_stop(); // todo stop & clear on task or external ??
            vTaskDelete(logic_analyzer_task_handle);
        }

        else
        {
#ifdef CONFIG_ANALYZER_USE_HI_LEVEL5_INTERRUPT
            ll_hi_level_triggered_isr_timeout_stop(); // restore gpio irq reg
#endif
            cfg->logic_analyzer_cb(NULL, 0, 0); // timeout
            logic_analyzer_stop();              // todo stop & clear on task or external ??
            vTaskDelete(logic_analyzer_task_handle);
        }
    }
}
/**
 * @brief Start logic analyzer
 *
 * @param config Configurations - see logic_analyzer_config_t struct
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM No memory to initialize logic_analyzer
 *     - ?????????????? - logic_analyzer already working (ESP_ERR_INVALID_ARG)
 *     - ESP_FAIL Initialize fail
 */
esp_err_t start_logic_analyzer(logic_analyzer_config_t *config)
{
    esp_err_t ret = 0;
    if (config->meashure_timeout == 0) // restart
    {
        if (logic_analyzer_started)
        {
#ifdef CONFIG_ANALYZER_USE_HI_LEVEL5_INTERRUPT
            ll_hi_level_triggered_isr_timeout_stop(); // restore gpio irq reg
#endif
            config->logic_analyzer_cb(NULL, 0, 0); // timeout or restart
            logic_analyzer_stop();                 // todo stop & clear on task or external ??
            vTaskDelete(logic_analyzer_task_handle);
        }
        else
        {
            config->logic_analyzer_cb(NULL, 0, 0); // timeout or restart
        }
        ret = ESP_OK;
        goto _retcode;
    }

    if (logic_analyzer_started)
    {
        return ESP_ERR_INVALID_STATE;
    }
    logic_analyzer_started = 1;
    // check cb pointer
    if (config->logic_analyzer_cb == NULL)
    {
        goto _ret;
    }

    // check GPIO num - 0-39 or num < 0
    for (int i = 0; i < LA_MAX_PIN; i++)
    {
        if (config->pin[i] > LA_MAX_GPIO)
        {
            goto _ret;
        }
    }
    if (config->pin_trigger > LA_MAX_GPIO)
    {
        goto _ret;
    }
    else if ((config->trigger_edge >= 0 && config->trigger_edge < GPIO_INTR_MAX) == 0)
    {
        goto _ret;
    }
    // check sample rate
    if (config->sample_rate > LA_MAX_SAMPLE_RATE || config->sample_rate < LA_MIN_SAMPLE_RATE)
    {
        goto _ret;
    }
    // check number of samples
    if (config->number_of_samples > LA_MAX_SAMPLE_CNT)
    {
        goto _ret;
    }
    // allocate frame buffer
#ifndef CONFIG_ANALYZER_USE_PSRAM
    // alloc on RAM
    uint32_t number_of_samples = config->number_of_samples & ~0x3; // burst transfer word align
    uint32_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    if (largest_free_block < number_of_samples * LA_BYTE_IN_SAMPLE)
    {
        number_of_samples = (largest_free_block / LA_BYTE_IN_SAMPLE) & ~0x3; // burst transfer word align
    }
    ESP_LOGD("DMA HEAP Before", "All_dma_heap=%d Largest_dma_heap_block=%d", heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    la_frame.fb.len = number_of_samples * LA_BYTE_IN_SAMPLE;
    la_frame.fb.buf = heap_caps_calloc(la_frame.fb.len, 1, MALLOC_CAP_DMA); // malloc ??
    ESP_LOGD("DMA HEAP After", "All_dma_heap=%d Largest_dma_heap_block=%d", heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
#endif
#ifdef CONFIG_ANALYZER_USE_PSRAM
    // test  - alloc on PSRAM -> on debug
    uint32_t number_of_samples = config->number_of_samples & ~0xf; // burst transfer 16 byte align
    uint32_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    if (largest_free_block < number_of_samples * LA_BYTE_IN_SAMPLE)
    {
        number_of_samples = (largest_free_block / LA_BYTE_IN_SAMPLE) & ~0xf; // burst transfer word align
    }
    ESP_LOGI("DMA PSRAM HEAP Before", "All_dma_heap=%d Largest_dma_heap_block=%d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM), heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    la_frame.fb.len = number_of_samples * LA_BYTE_IN_SAMPLE;
    la_frame.fb.buf = heap_caps_aligned_alloc(16, la_frame.fb.len, MALLOC_CAP_SPIRAM); // malloc ??
    memset(la_frame.fb.buf, 0, number_of_samples * LA_BYTE_IN_SAMPLE);
    ESP_LOGI("DMA PSRAM HEAP After", "All_dma_heap=%d Largest_dma_heap_block=%d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM), heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
#endif
    if (la_frame.fb.buf == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto _retcode;
    }
    //  allocate dma descriptor buffer
    la_frame.dma = allocate_dma_descriptors(la_frame.fb.len, la_frame.fb.buf);
    if (la_frame.dma == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto _freebuf_ret;
    }
    // configure   - pin definition, pin trigger, sample frame & dma frame, clock divider
    logic_analyzer_ll_config(config->pin, config->sample_rate, &la_frame);
    // start main task - check logic analyzer get data & call cb
    if (pdPASS != xTaskCreate(logic_analyzer_task, "la_task", LA_TASK_STACK * 4, config, configMAX_PRIORITIES - 2, &logic_analyzer_task_handle))
    {
        ret = ESP_ERR_NO_MEM;
        goto _freedma_ret;
    }
    // init dma eof isr
    ret = logic_analyzer_ll_init_dma_eof_isr(logic_analyzer_task_handle);
    if (ret != ESP_OK)
    {
        goto _freetask_ret;
    }
    /*
     * triggered  start
     */
    if (config->pin_trigger < 0)
    {
        logic_analyzer_ll_start();
    }
    else
    {
        logic_analyzer_ll_triggered_start(config->pin_trigger, config->trigger_edge);
    }
    return ESP_OK;

_freetask_ret:
    vTaskDelete(logic_analyzer_task_handle);
_freedma_ret:
    free(la_frame.dma);
_freebuf_ret:
    free(la_frame.fb.buf);
_retcode:
    logic_analyzer_started = 0;
    return ret;
_ret:
    logic_analyzer_started = 0;
    return ESP_ERR_INVALID_ARG;
}
