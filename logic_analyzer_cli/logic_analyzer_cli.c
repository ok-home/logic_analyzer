/* logic analyzer cli interface

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <string.h>

#include "logic_analyzer_cli.h"
#include "logic_analyzer_pin_definition.h"
#include "logic_analyzer_hal.h"
#include "logic_analyzer_const_definition.h"
#include "jsmn.h"

#define BUF_SIZE (1024)
#define JSON_SIZE (256)

static const char *TAG = "LOGIC ANALYZER CLI";

static void uart_send_string(char *str);

static logic_analyzer_config_t la_cfg = {
    .pin = {LA_PIN_0, LA_PIN_1, LA_PIN_2, LA_PIN_3, LA_PIN_4, LA_PIN_5, LA_PIN_6, LA_PIN_7, LA_PIN_8, LA_PIN_9, LA_PIN_10, LA_PIN_11, LA_PIN_12, LA_PIN_13, LA_PIN_14, LA_PIN_15},
    .pin_trigger = LA_PIN_TRIGGER,
    .trigger_edge = LA_PIN_EDGE,
    .number_of_samples = LA_SAMPLE_COUNT,
    .sample_rate = LA_SAMPLE_RATE,
    .number_channels = LA_ANALYZER_CHANNELS,
    .samples_to_psram = LA_ANALYZER_PSRAM,
    .meashure_timeout = LA_DEFAULT_TiMEOUT,
    .logic_analyzer_cb = NULL};

static logic_analyzer_hw_param_t la_hw;

static void logic_analyzer_cli_cb(uint8_t *sample_buf, int samples, int sample_rate, int channels)
{
    char jsonstr[64];
    if (samples) // data ready
    {
        int bytes_in_buff = channels > 4 ? (samples * (channels / 8)) : (samples / 2);
        sprintf(jsonstr, "{\"smp\":\"%d\"}\n", samples);
        uart_send_string(jsonstr);
        sprintf(jsonstr, "{\"clk\":\"%d\"}\n", sample_rate);
        uart_send_string(jsonstr);
        sprintf(jsonstr, "{\"chn\":\"%d\"}\n", channels);
        uart_send_string(jsonstr);
        ESP_LOGI(TAG, "Start samples transfer %d", samples);
        uart_send_string("Start samples transfer\n");

        for (int i = 0; i < bytes_in_buff; i++)
        {
            if (channels == 4)
            {
                char data = sample_buf[i] & 0xf;
                uart_write_bytes(CONFIG_ANALYZER_UART_PORT_NUMBER, (const char *)&data, 1);
                data = (sample_buf[i] >> 4) & 0xf;
                uart_write_bytes(CONFIG_ANALYZER_UART_PORT_NUMBER, (const char *)&data, 1);
            }
            else // 8 & 16  chann
            {
                uart_write_bytes(CONFIG_ANALYZER_UART_PORT_NUMBER, (const char *)&sample_buf[i], 1);
            }
        }

        ESP_LOGI(TAG, "Samples transfer done");
        uart_send_string("Samples transfer done\n");
    }
    else // timeout detected
    {
        ESP_LOGE(TAG, "Error - callback timeout deteсted");
        uart_send_string("Error - callback timeout deteсted\n");
    }
}

// simple json parse -> only one parametr name/val
static esp_err_t json_to_str_parm(char *jsonstr, char *nameStr, char *valStr) // распаковать строку json в пару  name/val
{
    int r; // количество токенов
    jsmn_parser p;
    jsmntok_t t[5]; // только 2 пары параметров и obj

    jsmn_init(&p);
    r = jsmn_parse(&p, jsonstr, strlen(jsonstr), t, sizeof(t) / sizeof(t[0]));
    if (r < 2)
    {
        valStr[0] = 0;
        nameStr[0] = 0;
        return ESP_FAIL;
    }
    strncpy(nameStr, jsonstr + t[2].start, t[2].end - t[2].start);
    nameStr[t[2].end - t[2].start] = 0;
    if (r > 3)
    {
        strncpy(valStr, jsonstr + t[4].start, t[4].end - t[4].start);
        valStr[t[4].end - t[4].start] = 0;
    }
    else
        valStr[0] = 0;
    return ESP_OK;
}

static void uart_read_json(char *json_string)
{
    char send_str[64];
    char name[16];
    char val[16];
    esp_err_t ret = 0;

    // parse json from UART
    if (json_to_str_parm(json_string, name, val) == ESP_OK)
    {
        if (strncmp("pin", name, 3) == 0) // gpio pins
        {
            int pin_numb = atoi(name + 3);
            int gpio = atoi(val);
            la_cfg.pin[pin_numb] = gpio;
        }
        else if (strncmp("trg", name, 3) == 0) // trigg pin
        {
            la_cfg.pin_trigger = atoi(val);
        }
        else if (strncmp("edg", name, 3) == 0) // trigg edge
        {
            la_cfg.trigger_edge = atoi(val);
        }
        else if (strncmp("smp", name, 3) == 0) // sample count
        {
            la_cfg.number_of_samples = atoi(val);
        }
        else if (strncmp("clk", name, 3) == 0) // sample rate
        {
            la_cfg.sample_rate = atoi(val);
        }
        else if (strncmp("chn", name, 3) == 0) // channels
        {
            la_cfg.number_channels = atoi(val);
        }
        else if (strncmp("ram", name, 3) == 0) // psram
        {
            la_cfg.samples_to_psram = atoi(val);
        }
        else if (strncmp("tmo", name, 3) == 0) // timeout
        {
            la_cfg.meashure_timeout = atoi(val) > 0 ? atoi(val) * 100 : atoi(val);
        }
    }
    else
    {
        if (strncmp("endcfg", json_string, 6) == 0) // endcfg - start LA
        {
            la_cfg.logic_analyzer_cb = logic_analyzer_cli_cb;
            ret = start_logic_analyzer(&la_cfg);
            if (ret)
            {
                ESP_LOGE(TAG, "Start logic analyzer error %x", ret);
                uart_send_string("Start logic analyzer error\n");
            }
            else
            {
                ESP_LOGI(TAG, "Start logic analyzer OK");
                uart_send_string("Start logic analyzer OK\n");
            }
        }
        else if (strncmp("getcfg", json_string, 6) == 0)
        {
            la_hw.current_channels = la_cfg.number_channels;
            la_hw.current_psram = la_cfg.samples_to_psram;
            logic_analyzer_get_hw_param(&la_hw); // get HW params
            sprintf(send_str, "Hardware sample rate and sample count for %d channel and %s\n", la_hw.current_channels, la_hw.current_psram ? "Psram" : "Ram");
            uart_send_string(send_str);
            sprintf(send_str, "   Available sample rate min %d max %d\n", la_hw.min_sample_rate, la_hw.max_sample_rate);
            uart_send_string(send_str);
            sprintf(send_str, "   Available sample count min %d max %d\n", la_hw.min_sample_cnt, la_hw.max_sample_cnt);
            uart_send_string(send_str);
            sprintf(send_str, "Available channels min %d max %d\n", la_hw.min_channels, la_hw.max_channels);
            uart_send_string(send_str);
            sprintf(send_str, "Available %s\n", la_hw.available_psram ? "Psram and Ram" : "Ram only");
            uart_send_string(send_str);
        }
        else
        {
            ESP_LOGE(TAG, "Receive undefined string %s", json_string);
        }
    }
}

static void uart_send_string(char *str)
{
    uart_write_bytes(CONFIG_ANALYZER_UART_PORT_NUMBER, str, strlen(str));
}
static int uart_read_line(char *buff, size_t size)
{
    int idx = 0;
    while (uart_read_bytes(CONFIG_ANALYZER_UART_PORT_NUMBER, &buff[idx], 1, portMAX_DELAY))
    {
        if (buff[idx] == '\n' || buff[idx] == 0 || idx == size - 1)
            break;
        idx++;
    }
    // idx++;
    buff[idx] = 0;
    return idx;
}

static void logic_analyzer_cli_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = CONFIG_ANALYZER_UART_PORT_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ANALYZER_UART_PORT_NUMBER, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_ANALYZER_UART_PORT_NUMBER, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_ANALYZER_UART_PORT_NUMBER, CONFIG_ANALYZER_UART_PORT_TX_PIN, CONFIG_ANALYZER_UART_PORT_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    char data_str[JSON_SIZE] = {0};
    while (1)
    {
        int len = uart_read_line(data_str, JSON_SIZE);
        if (len)
        {
            uart_read_json(data_str);
        }
    }
}
void logic_analyzer_cli(void)
{
    xTaskCreate(logic_analyzer_cli_task, "la_cli_task", 4096*2, NULL, 10, NULL);
}

