/* logic analizer hal example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "logic_analizer_hal.h"
#include "logic_analizer_ll.h"

// frame buff & dma descripter
static la_frame_t la_frame = {
    .fb.buf = NULL,
    .fb.len = 0, // bytes *** one sample = 2 byte
    .dma = NULL};

static TaskHandle_t logic_analizer_task_handle = 0; // main task handle
static int logic_analizer_started = 0;              // flag start dma

/**
 * @brief allocate dma descriptor
 *
 * @param uint16_t size - size of sample frame buffer (bytes)
 * @param uint8_t *buffer - pointer of sample frame buffer
 *
 * @return
 *     - dma descriptor ( NULL if no mem )
 */
static lldesc_t *allocate_dma_descriptors(uint16_t size, uint8_t *buffer)
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
static void logic_analizer_stop(void)
{
    // stop dma transfer
    logic_analizer_ll_stop();
    // deinit dma isr
    logic_analizer_ll_deinit_dma_eof_isr();
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
    logic_analizer_started = 0;
}
/**
 *  @brief main logic analizer task
 *       call callback function aftr dma transfer
 *       callback param = 0 if timeout
 */
static void logic_analizer_task(void *arg)
{
    int noTimeout;
    logic_analizer_config_t *cfg = (logic_analizer_config_t *)arg;

    while (1)
    {
        noTimeout = ulTaskNotifyTake(pdFALSE, cfg->meashure_timeout); // portMAX_DELAY
        if (noTimeout)
        {
            // dma data ready
            // sample sequence in 32 word - adr0=sample1, adr1=sample0 
            // swap sample sequence if necessary
            // for sump sample swap on uart tx
            cfg->logic_analizer_cb((uint16_t *)la_frame.fb.buf, la_frame.fb.len / 2, logic_analizer_ll_get_sample_rate(cfg->sample_rate));
            logic_analizer_stop(); // todo stop & clear on task or external ??
            vTaskDelete(logic_analizer_task_handle);
        }

        else
        {
            cfg->logic_analizer_cb(NULL, 0, 0); // timeout
            logic_analizer_stop();              // todo stop & clear on task or external ??
            vTaskDelete(logic_analizer_task_handle);
        }
    }
}
/**
 * @brief Start logic analizer
 *
 * @param config Configurations - see logic_analizer_config_t struct
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM No memory to initialize logic_analizer
 *     - ?????????????? - logic_analizer already working (ESP_ERR_INVALID_ARG)
 *     - ESP_FAIL Initialize fail
 */
esp_err_t start_logic_analizer(logic_analizer_config_t *config)
{
    esp_err_t ret = 0;
    if (logic_analizer_started)
        return 9;//ESP_ERR_INVALID_ARG; // todo change err code

    logic_analizer_started = 1;
    // check cb pointer
    if (config->logic_analizer_cb == NULL)
    {
        ret = ESP_ERR_INVALID_ARG;
        goto _ret;
    }

    // check GPIO num - 0-39 or num < 0
    for (int i = 0; i < 16; i++)
    {
        if (config->pin[i] > 39)
        {
            ret = ESP_ERR_INVALID_ARG;
            goto _ret;
        }
    }
    if (config->pin_trigger > 39)
    {
        ret = ESP_ERR_INVALID_ARG;
        goto _ret;
    }
    else if ((config->trigger_edge >= 0 && config->trigger_edge < GPIO_INTR_MAX) == 0)
    {
        ret = ESP_ERR_INVALID_ARG;
        goto _ret;
    }
    // check sample rate
    if (config->sample_rate > LA_MAX_SAMPLE_RATE || config->sample_rate < LA_MIN_SAMPLE_RATE)
    {
        ret = ESP_ERR_INVALID_ARG;
        goto _ret;
    }
    // check number of samples
    if (config->number_of_samples >= 32768)
    {
        ret = ESP_ERR_INVALID_ARG;
        goto _ret;
    }
    // allocate frame buffer
    la_frame.fb.len = config->number_of_samples * 2;
    la_frame.fb.buf = heap_caps_calloc(la_frame.fb.len,1, MALLOC_CAP_DMA);
    if (la_frame.fb.buf == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto _ret;
    }
    //  allocate dma descriptor buffer
    la_frame.dma = allocate_dma_descriptors(la_frame.fb.len, la_frame.fb.buf);
    if (la_frame.dma == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto _freebuf_ret;
    }
    // configure I2S  - pin definition, pin trigger, sample frame & dma frame, clock divider
    logic_analizer_ll_config(config->pin, config->pin_trigger, config->trigger_edge, config->sample_rate, &la_frame);
    // start main task - check logic analizer get data & call cb
    if (pdPASS != xTaskCreate(logic_analizer_task, "la_task", LA_TASK_STACK*4, config, configMAX_PRIORITIES - 2, &logic_analizer_task_handle))
    {
        ret = ESP_ERR_NO_MEM;
        goto _freedma_ret;
    }
    // init dma eof isr
    ret = logic_analizer_ll_init_dma_eof_isr(logic_analizer_task_handle);
    if (ret != ESP_OK)
    {
        goto _freetask_ret;
    }
    /*
     * triggered  start
     */
    if (config->pin_trigger < 0)
    {
        logic_analizer_ll_start();
    }
    else
    {
        logic_analizer_ll_triggered_start(config->pin_trigger);
    }
    return ESP_OK;

_freetask_ret:
    vTaskDelete(logic_analizer_task_handle);
_freedma_ret:
    free(la_frame.dma);
_freebuf_ret:
    free(la_frame.fb.buf);
_ret:
    logic_analizer_started = 0;
    return ret;
}
