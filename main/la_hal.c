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
    .fb.len = 1024,
    .dma = NULL};

TaskHandle_t la_task_handle = 0;
void la_task(void *p)
{
    while(1)
    {
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY); 
        printf("generate isr - transfer done\n");
    }
}

static lldesc_t *allocate_dma_descriptors(uint32_t count, uint16_t size, uint16_t *buffer)
{
    /*lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (dma == NULL) {
        return dma;
    }

    for (int x = 0; x < count; x++) {
        dma[x].size = size;
        dma[x].length = 0;
        dma[x].sosf = 0;
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);
        dma[x].empty = (uint32_t)&dma[(x + 1) % count];
    }
    */
    // test only one frame
    count = 1;
    size = 2048;

    lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    dma[0].size = size & 0xfff;
    dma[0].length = 8 & 0xfff;
    dma[0].sosf = 0;
    dma[0].eof = 1;
    dma[0].owner = 1;
    dma[0].buf = (uint8_t*)(buffer);
    dma[0].empty = 0;

    return dma;
}

static esp_err_t la_dma_config(la_frame_t *cfg)
{
    cfg->fb.buf = heap_caps_malloc(cfg->fb.len, MALLOC_CAP_DMA);
    if (cfg->fb.buf == NULL)
        return ESP_FAIL;

    cfg->dma = allocate_dma_descriptors(1, cfg->fb.len, cfg->fb.buf);

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

    memset(la_frame.fb.buf, 0x33, 32);
    printf("%d\n", la_frame.fb.len);
    for (int i = 0; i < 32; i++)
    {
        printf("%x ", la_frame.fb.buf[i]);
    }
    printf("\n");

    la_ll_start(&la_frame);

    printf("done\n");
    vTaskDelay(100);
    printf("%d\n", la_frame.fb.len);
    for (int i = 0; i < 32; i++)
    {
        printf("%x ", la_frame.fb.buf[i]);
    }
    printf("\n");
}
