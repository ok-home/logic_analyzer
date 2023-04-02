// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "la.h"
// #include "la_ll.h"
// #include "la_hal.h"
#include "esp_timer.h"
#include "esp32/rom/ets_sys.h" // will be removed in idf v5.0

#define LA_TASK_STACK (2 * 1024)

static const char *TAG = "la_hal";

la_config_t la_cfg = {
    //.pin_xclk = LA_PIN_XCLK,
    //.pin_pclk = LA_PIN_PCLK,
    .pin_d0 = LA_PIN_D0,
    .pin_d1 = LA_PIN_D1,
    .pin_d2 = LA_PIN_D2,
    .pin_d3 = LA_PIN_D3,
    .pin_d4 = LA_PIN_D4,
    .pin_d5 = LA_PIN_D5,
    .pin_d6 = LA_PIN_D6,
    .pin_d7 = LA_PIN_D7,
    //.pin_vsync = LA_PIN_VSYNC,
    //.pin_href = LA_PIN_HREF,

    //.xclk_freq_hz = 10000000,
    //.pclk_freq_hz = 2000000,

    //.frame_size = 1024

};

la_frame_t la_frame = {
    .fb.buf = NULL,
    .fb.len = 4096 * 2 - 1024, // bytes *** one sample = 2 byte
    .dma = NULL};

TaskHandle_t la_task_handle = 0;
void la_task(void *p)
{
    while (1)
    {
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        printf("generate isr - transfer done\n");
        printf("%d\n", la_frame.fb.len);
        for (int i = 0; i < la_frame.fb.len + 16; i++)
        {
            if (i % 16 == 0)
                printf("\n %d ", i);
            printf("%x ", la_frame.fb.buf[i]);
        }
        printf("\n");
    }
}
#define DMA_FRAME 4092

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

static esp_err_t la_dma_config(la_frame_t *cfg)
{
    cfg->fb.buf = heap_caps_malloc(cfg->fb.len, MALLOC_CAP_DMA);
    if (cfg->fb.buf == NULL)
        return ESP_FAIL;

    cfg->dma = allocate_dma_descriptors(cfg->fb.len, cfg->fb.buf);

    return ESP_OK;
}

esp_err_t la_init(la_config_t *config)
{
    la_ll_set_pin(config);
    la_ll_config();
    ESP_LOGI(TAG, "cam init ok");
    return ESP_OK;
}

esp_err_t la_config(la_frame_t *config)
{

    la_dma_config(config);
    xTaskCreate(la_task, "la_task", LA_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &la_task_handle);
    la_ll_init_isr(la_task_handle);
    ESP_LOGI(TAG, "cam config ok");
    return ESP_OK;
}

esp_err_t la_deinit(void)
{
    la_stop();
    /*    if (cam_obj->task_handle) {
            vTaskDelete(cam_obj->task_handle);
        }
        if (cam_obj->event_queue) {
            vQueueDelete(cam_obj->event_queue);
        }
        if (cam_obj->frame_buffer_queue) {
            vQueueDelete(cam_obj->frame_buffer_queue);
        }

        ll_cam_deinit(cam_obj);

        if (cam_obj->dma) {
            free(cam_obj->dma);
        }
        if (cam_obj->dma_buffer) {
            free(cam_obj->dma_buffer);
        }
        free(cam_obj);
        cam_obj = NULL;
        */
    return ESP_OK;
}

void la_stop(void)
{
    la_ll_stop();
}

void la_start(void)
{
    la_init(&la_cfg);
    la_config(&la_frame);
    la_ll_start(&la_frame);

    /*
        vTaskDelay(100);
        printf("%d\n", la_frame.fb.len);
        for (int i = 0; i < la_frame.fb.len+16; i++)
        {
            if(i%16==0) printf("\n %d ",i);
            printf("%x ", la_frame.fb.buf[i]);

        }
        printf("\n");
        */
}
