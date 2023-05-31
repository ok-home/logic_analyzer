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

extern hi_interrupt_state_t la_hi_interrupt_state;

#define LA_TASK_STACK 2048
#define DMA_FRAME 4092

// frame buff & dma descripter
static la_frame_t la_frame = {
    .fb.buf = NULL,
    .fb.len = 0, // bytes *** one sample = 2 byte
    .dma = NULL};

static TaskHandle_t logic_analyzer_task_handle = 0; // main task handle
static int logic_analyzer_started = 0;              // flag start dma
//
// sample sequence in 32 word - adr0=sample1, adr1=sample0 
// swap sample sequence
//
static inline void swap_buf(uint16_t* buf,int cnt){
    uint16_t tmp;
    for( int i=0;i<cnt;i+=2)
    {
        tmp = buf[i];
        buf[i]=buf[i+1];
        buf[i+1]=tmp;
    }
}

/**
 * @brief allocate dma descriptor
 *
 * @param uint16_t size - size of sample frame buffer (bytes)
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
            // sample sequence in 32 word - adr0=sample1, adr1=sample0 
            // swap sample sequence
            swap_buf((uint16_t *)la_frame.fb.buf,la_frame.fb.len / 2);
            cfg->logic_analyzer_cb((uint16_t *)la_frame.fb.buf, la_frame.fb.len / 2, logic_analyzer_ll_get_sample_rate(cfg->sample_rate));
            logic_analyzer_stop(); // todo stop & clear on task or external ??
            vTaskDelete(logic_analyzer_task_handle);
        }

        else
        {
            ll_hi_level_triggered_isr_timeout_stop(); // restore gpio irq reg

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
    if(config->meashure_timeout==0)  // restart
    {
       if (logic_analyzer_started )  
       {
            ll_hi_level_triggered_isr_timeout_stop(); // restore gpio irq reg
            config->logic_analyzer_cb(NULL, 0, 0); // timeout or restart
            logic_analyzer_stop();              // todo stop & clear on task or external ??
            vTaskDelete(logic_analyzer_task_handle);
      }
       else
       {
            config->logic_analyzer_cb(NULL, 0, 0); // timeout or restart
       }
            ret = ESP_OK;
            goto _retcode;

    }

    if (logic_analyzer_started ) 
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
    for (int i = 0; i < 16; i++)
    {
        if (config->pin[i] > 39)
        {
            goto _ret;
        }
    }
    if (config->pin_trigger > 39)
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
    // check number of samples // remove - auto max sample block
    //if (config->number_of_samples >= 70000)// 32768)
    //{
    //    goto _ret;
    //}
    // allocate frame buffer
    // test  - check maximum available buffer size
    uint32_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    if(largest_free_block < config->number_of_samples * 2)
        {config->number_of_samples=(largest_free_block/4)*2;}
    ESP_LOGI("DMA HEAP Before","All_dma_heap=%d Largest_dma_heap_block=%d",heap_caps_get_free_size(MALLOC_CAP_DMA),heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    la_frame.fb.len = config->number_of_samples * 2;
    la_frame.fb.buf = heap_caps_calloc(la_frame.fb.len,1, MALLOC_CAP_DMA);
    if (la_frame.fb.buf == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto _retcode;
    }
    ESP_LOGI("DMA HEAP After","All_dma_heap=%d Largest_dma_heap_block=%d",heap_caps_get_free_size(MALLOC_CAP_DMA),heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    //  allocate dma descriptor buffer
    la_frame.dma = allocate_dma_descriptors(la_frame.fb.len, la_frame.fb.buf);
    if (la_frame.dma == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto _freebuf_ret;
    }
    ESP_LOGI("DMA HEAP After Descr","All_dma_heap=%d Largest_dma_heap_block=%d",heap_caps_get_free_size(MALLOC_CAP_DMA),heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    // configure I2S  - pin definition, pin trigger, sample frame & dma frame, clock divider
    logic_analyzer_ll_config(config->pin,config->sample_rate, &la_frame);
    // start main task - check logic analyzer get data & call cb
    if (pdPASS != xTaskCreate(logic_analyzer_task, "la_task", LA_TASK_STACK*4, config, configMAX_PRIORITIES - 2, &logic_analyzer_task_handle))
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
        logic_analyzer_ll_triggered_start(config->pin_trigger,config->trigger_edge);
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
