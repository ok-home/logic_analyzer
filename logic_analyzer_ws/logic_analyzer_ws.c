#include <esp_log.h>
#include <esp_http_server.h>
#include <freertos/queue.h>

#include "logic_analyzer_const_definition.h"
#include "logic_analyzer_pin_definition.h"
#include "logic_analyzer_hal.h"

#include "logic_analyzer_ws.h"
#include "logic_analyzer_ws_html.h"
#include "jsmn.h"

static const char *TAG = "LA_WS";

#define JSON_QUEUE_LEN 64
#define JSON_QUEUE_SIZE 4

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

typedef struct async_resp_arg
{
    httpd_handle_t hd;
    int fd;
} async_resp_arg_t;

static async_resp_arg_t ra;
static TaskHandle_t draw_html_handle = 0;
static TaskHandle_t read_json_handle = 0;
static QueueHandle_t read_json_queue = 0;
static esp_err_t send_ws_string(const char *json_string);
static esp_err_t send_ws_bin(const uint8_t *data, int len);

// dma transfer end. bin data ready to ws send
static void logic_analyzer_cb(uint8_t *sample_buf, int samples, int sample_rate, int channels)
{
    char jsonstr[64];
    esp_err_t ret = 0;
    if (samples) // data ready
    {
        int l_samples = channels > 4 ? (samples * (channels / 8)) : (samples / 2);
        sprintf(jsonstr, "{\"rowID\":\"%s%02d\",\"rowVal\":\"%d\"}", rowID[ROW_MSMP], 0, samples);
        ret = send_ws_string(jsonstr);
        sprintf(jsonstr, "{\"rowID\":\"%s%02d\",\"rowVal\":\"%d\"}", rowID[ROW_MCLK], 0, sample_rate);
        ret = send_ws_string(jsonstr);
        ESP_LOGI(TAG, "Start samples transfer %d", l_samples);
        send_ws_string("Start samples transfer");

        ret = send_ws_bin((const uint8_t *)sample_buf, l_samples);
        if (ret)
        {
            ESP_LOGE(TAG, "Samples transfer err %d", ret);
            send_ws_string("Samples transfer err");
            return;
        }
        ESP_LOGI(TAG, "Samples transfer done");
        send_ws_string("Samples transfer done");
    }
    else // timeout detected
    {
        ESP_LOGE(TAG, "Error - callback imeout deteсted");
        send_ws_string("Error - callback imeout deteсted");
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
static esp_err_t send_ws_string(const char *string)
{
    esp_err_t ret;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = (uint8_t *)string;
    ws_pkt.len = strlen((char *)ws_pkt.payload);
    ret = httpd_ws_send_data(ra.hd, ra.fd, &ws_pkt);
    if (ret)
    {
        ESP_LOGE(TAG, "err ws_send_data string %d", ret);
    }
    return ret;
}
static esp_err_t send_ws_bin(const uint8_t *data, int len)
{
    esp_err_t ret = 0;
    httpd_ws_frame_t ws_pkt;
    int bytes_to_send = len;
    int bytes_in_frame = 2048; // 1024;
    uint8_t *buf = (uint8_t *)data;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    while (bytes_to_send > bytes_in_frame)
    {
        ws_pkt.fragmented = true;
        ws_pkt.len = bytes_in_frame;
        ws_pkt.payload = buf;
        ws_pkt.final = false; // fragmented
        ret = httpd_ws_send_data(ra.hd, ra.fd, &ws_pkt);
        if (ret)
        {
            return ret;
        }
        buf += bytes_in_frame;
        bytes_to_send -= bytes_in_frame;
        ws_pkt.type = HTTPD_WS_TYPE_CONTINUE;
    }
    ws_pkt.len = bytes_to_send;
    ws_pkt.payload = buf;
    ws_pkt.final = true; // last fragment
    ret = httpd_ws_send_data(ra.hd, ra.fd, &ws_pkt);
    return ret;
}

static esp_err_t draw_html_datalist(void)
{
    char jsonstr[128];
    esp_err_t ret = 0;
    for (int i = 0; i < DATALIST_MAX; i++)
    {
        sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\"}",
                hdrID[HDR_DATALIST], rowID[ROW_LST], i);
        ret = send_ws_string(jsonstr);
    }
    return ret;
}

static esp_err_t option_to_html(const options_list_t *list, int min, int max)
{
    char jsonstr[128];
    esp_err_t ret = 0;
    while (list->datalist[0])
    {
        if (list->value >= min && list->value <= max)
        {
            sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s\",\"rowVal\":\"%ld\",\"rowLbl\":\"%s\"}",
                    hdrID[HDR_OPTIONS], list->datalist, list->value, list->label);
            ret = send_ws_string(jsonstr);
        }
        list++;
    }
    return ret;
}
static esp_err_t draw_html_options(void)
{
    esp_err_t ret = 0;
    ret = option_to_html(pin_options, MIN_GPIO, MAX_GPIO);
    ret = option_to_html(edge_options, -1, 5);
    ret = option_to_html(sample_options, la_hw.min_sample_cnt, la_hw.max_sample_cnt);
    ret = option_to_html(clk_options, la_hw.min_sample_rate, la_hw.max_sample_rate);
    ret = option_to_html(timeout_options, -1, 120);
    ret = option_to_html(channel_options, la_hw.min_channels, la_hw.max_channels);
    ret = option_to_html(psram_options, 0, la_hw.available_psram);
    return ret;
}
static esp_err_t draw_html_config(void)
{
    char jsonstr[256];
    esp_err_t ret = 0;
    for (int i = 0; i < la_cfg.number_channels; i++)
    {
        sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s%02d\",\"rowType\":\"%s\",\"rowMax\":\"%d\",\"rowMin\":\"%d\",\"rowVal\":\"%d\",\"rowStep\":\"%d\",\"rowDis\":\"%d\",\"rowList\":\"%s%02d\",\"rowEvent\":\"%d\"}",
                hdrID[HDR_CONFIG], rowID[ROW_PIN], i, rowLbl[ROW_LBL_PIN], i, rowType[ROW_NUMBER], MAX_GPIO, MIN_GPIO, la_cfg.pin[i], 1, 0, rowID[ROW_LST], DATALIST_PIN, CHECK_CFG_DATA_EVENT);
        ret = send_ws_string(jsonstr);
    }
    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowMax\":\"%d\",\"rowMin\":\"%d\",\"rowVal\":\"%d\",\"rowStep\":\"%d\",\"rowDis\":\"%d\",\"rowList\":\"%s%02d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_CONFIG], rowID[ROW_TRG], 0, rowLbl[ROW_LBL_TRIG], rowType[ROW_NUMBER], MAX_GPIO, MIN_GPIO, la_cfg.pin_trigger, 1, 0, rowID[ROW_LST], DATALIST_PIN, CHECK_CFG_DATA_EVENT);
    ret = send_ws_string(jsonstr);

    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowMax\":\"%d\",\"rowMin\":\"%d\",\"rowVal\":\"%d\",\"rowStep\":\"%d\",\"rowDis\":\"%d\",\"rowList\":\"%s%02d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_CONFIG], rowID[ROW_EDG], 0, rowLbl[ROW_LBL_EDGE], rowType[ROW_NUMBER], 2, 1, la_cfg.trigger_edge, 1, 0, rowID[ROW_LST], DATALIST_EDGE, CHECK_CFG_DATA_EVENT);
    ret = send_ws_string(jsonstr);

    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowMax\":\"%d\",\"rowMin\":\"%d\",\"rowVal\":\"%d\",\"rowStep\":\"%d\",\"rowDis\":\"%d\",\"rowList\":\"%s%02d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_CONFIG], rowID[ROW_SMP], 0, rowLbl[ROW_LBL_SAMPLE], rowType[ROW_NUMBER], la_hw.max_sample_cnt, la_hw.min_sample_cnt, la_cfg.number_of_samples, 100, 0, rowID[ROW_LST], DATALIST_SAMPLE, CHECK_CFG_DATA_EVENT);
    ret = send_ws_string(jsonstr);

    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowMax\":\"%d\",\"rowMin\":\"%d\",\"rowVal\":\"%d\",\"rowStep\":\"%d\",\"rowDis\":\"%d\",\"rowList\":\"%s%02d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_CONFIG], rowID[ROW_CLK], 0, rowLbl[ROW_LBL_CLOCK], rowType[ROW_NUMBER], la_hw.max_sample_rate, la_hw.min_sample_rate, la_cfg.sample_rate, 1000, 0, rowID[ROW_LST], DATALIST_CLK, CHECK_CFG_DATA_EVENT);
    ret = send_ws_string(jsonstr);
    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowMax\":\"%d\",\"rowMin\":\"%d\",\"rowVal\":\"%d\",\"rowStep\":\"%d\",\"rowDis\":\"%d\",\"rowList\":\"%s%02d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_CONFIG], rowID[ROW_TIMEOUT], 0, rowLbl[ROW_LBL_TIMEOUT], rowType[ROW_NUMBER], 50, -1,
            la_cfg.meashure_timeout > 0 ? la_cfg.meashure_timeout / 100 : la_cfg.meashure_timeout, // TICK = 10 MSek
            1, 0, rowID[ROW_LST], DATALIST_TIMEOUT, CHECK_CFG_DATA_EVENT);
    ret = send_ws_string(jsonstr);

    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowMax\":\"%d\",\"rowMin\":\"%d\",\"rowVal\":\"%d\",\"rowStep\":\"%d\",\"rowDis\":\"%d\",\"rowList\":\"%s%02d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_CONFIG], rowID[ROW_CHANNELS], 0, rowLbl[ROW_LBL_CHANNELS], rowType[ROW_NUMBER], la_hw.max_channels, la_hw.min_channels, la_cfg.number_channels, 1, 0, rowID[ROW_LST], DATALIST_CHANNELS, GET_HW_PARAM);
    ret = send_ws_string(jsonstr);
    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowMax\":\"%d\",\"rowMin\":\"%d\",\"rowVal\":\"%d\",\"rowStep\":\"%d\",\"rowDis\":\"%d\",\"rowList\":\"%s%02d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_CONFIG], rowID[ROW_PSRAM], 0, rowLbl[ROW_LBL_PSRAM], rowType[ROW_NUMBER], la_hw.available_psram, 0, la_cfg.samples_to_psram, 1, 0, rowID[ROW_LST], DATALIST_PSRAM, GET_HW_PARAM);
    ret = send_ws_string(jsonstr);

    return ret;
}
static esp_err_t draw_html_start(void)
{
    char jsonstr[256];
    esp_err_t ret = 0;

    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowVal\":\"%s\",\"rowDis\":\"%d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_START], rowID[ROW_START], 0, "", rowType[ROW_BUTTON], rowLbl[ROW_LBL_START], 0, START_EVENT);
    ret = send_ws_string(jsonstr);

    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowVal\":\"%s\",\"rowDis\":\"%d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_START], rowID[ROW_ZOOM], 0, "", rowType[ROW_BUTTON], rowLbl[ROW_LBL_ZOOM], 0, ZOOM_EVENT);
    ret = send_ws_string(jsonstr);

    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowVal\":\"%s\",\"rowDis\":\"%d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_START], rowID[ROW_SAVE], 0, "", rowType[ROW_BUTTON], rowLbl[ROW_LBL_SAVE], 0, SAVE_EVENT);
    ret = send_ws_string(jsonstr);

    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowDis\":\"%d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_START], rowID[ROW_MSMP], 1, rowLbl[ROW_LBL_MEASH_SAMPLES], rowType[ROW_TEXT], 1, NO_EVENT);
    ret = send_ws_string(jsonstr);

    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowDis\":\"%d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_START], rowID[ROW_MCLK], 1, rowLbl[ROW_LBL_MEASH_CLOCK], rowType[ROW_TEXT], 1, NO_EVENT);
    ret = send_ws_string(jsonstr);

    return ret;
}
// build html page
static void logic_analyzer_draw_html(void *arg)
{
    la_hw.current_channels = la_cfg.number_channels;
    la_hw.current_psram = la_cfg.samples_to_psram;
    logic_analyzer_get_hw_param(&la_hw); // get HW params

    draw_html_datalist(); // build datalist div
    draw_html_options();  // build options div
    draw_html_config();   // build config div
    draw_html_start();    // build start div
    vTaskDelete(NULL);
}
// main command loop from HTML page, data from ws
static void logic_analyzer_read_json(void *arg)
{
    char json_string[JSON_QUEUE_LEN];
    char name[16];
    char val[16];
    esp_err_t ret = 0;

    read_json_queue = xQueueCreate(JSON_QUEUE_SIZE, JSON_QUEUE_LEN);
    if (read_json_queue == NULL)
    {
        ESP_LOGE(TAG, "ERR Create json Queue");
        vTaskDelete(NULL);
    }
    while (1)
    {
        xQueueReceive(read_json_queue, json_string, portMAX_DELAY);
        // parse json from ws
        if (json_to_str_parm(json_string, name, val) == ESP_OK)
        {
            if (strncmp(rowID[ROW_PIN], name, 3) == 0) // gpio pins
            {
                int pin_numb = atoi(name + 3);
                int gpio = atoi(val);
                la_cfg.pin[pin_numb] = gpio;
            }
            else if (strncmp(rowID[ROW_TRG], name, 3) == 0) // trigg pin
            {
                la_cfg.pin_trigger = atoi(val);
            }
            else if (strncmp(rowID[ROW_EDG], name, 3) == 0) // trigg edge
            {
                la_cfg.trigger_edge = atoi(val);
            }
            else if (strncmp(rowID[ROW_SMP], name, 3) == 0) // sample count
            {
                la_cfg.number_of_samples = atoi(val);
            }
            else if (strncmp(rowID[ROW_CLK], name, 3) == 0) // sample rate
            {
                la_cfg.sample_rate = atoi(val);
            }
            else if (strncmp(rowID[ROW_CHANNELS], name, 3) == 0) // channels
            {
                la_cfg.number_channels = atoi(val);
            }
            else if (strncmp(rowID[ROW_PSRAM], name, 3) == 0) // psram
            {
                la_cfg.samples_to_psram = atoi(val);
            }
            else if (strncmp(rowID[ROW_TIMEOUT], name, 3) == 0) // timeout
            {
                la_cfg.meashure_timeout = atoi(val) > 0 ? atoi(val) * 100 : atoi(val);
            }
        }
        else
        {
            if (strncmp(END_CFG_MSG, json_string, 6) == 0) // endcfg - start LA
            {
                la_cfg.logic_analyzer_cb = logic_analyzer_cb;
                ret = start_logic_analyzer(&la_cfg);
                if (ret)
                {
                    ESP_LOGE(TAG, "Start logic analyzer error %x", ret);
                    send_ws_string("Start logic analyzer error");
                }
                else
                {
                    ESP_LOGI(TAG, "Start logic analyzer OK");
                    send_ws_string("Start logic analyzer OK");
                }
            }
            else if (strncmp(END_HW_REQ_MSG, json_string, 6) == 0)
            {
                send_ws_string(REDRAW_MSG);
            }
            else if (strncmp(CLEAR_DIV_MSG, json_string, 6) == 0)
            {
                la_hw.current_channels = la_cfg.number_channels;
                la_hw.current_psram = la_cfg.samples_to_psram;
                logic_analyzer_get_hw_param(&la_hw); // get HW params
                draw_html_datalist();                // build datalist div
                draw_html_options();                 // build options div
                draw_html_config();                  // build config div
            }
            else
            {
                ESP_LOGE(TAG, "Receive undefined string %s", json_string);
            }
        }
    }
}
// http server send html template & js
static esp_err_t logic_analyzer_get_handler(httpd_req_t *req)
{
    extern const unsigned char logic_analyzer_ws_html_start[] asm("_binary_logic_analyzer_ws_html_start");
    extern const unsigned char logic_analyzer_ws_html_end[] asm("_binary_logic_analyzer_ws_html_end");
    const size_t logic_analyzer_ws_html_size = (logic_analyzer_ws_html_end - logic_analyzer_ws_html_start);
    httpd_resp_send_chunk(req, (const char *)logic_analyzer_ws_html_start, logic_analyzer_ws_html_size);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}
// main ws handle read ws message & send to queue
static esp_err_t logic_analyzer_ws_handler(httpd_req_t *req)
{
    esp_err_t ret = 0;
    if (req->method == HTTP_GET) // handshake
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened %d", httpd_req_to_sockfd(req));
        // delete task & queue ? reopen ws
        if (read_json_queue)
        {
            vQueueDelete(read_json_queue);
            read_json_queue = 0;
        }
        if (read_json_handle)
        {
            vTaskDelete(read_json_handle);
            read_json_handle = 0;
        }

        ra.hd = req->handle;
        ra.fd = httpd_req_to_sockfd(req);
        ret = xTaskCreate(logic_analyzer_read_json, "read", 4096, (void *)&ra, uxTaskPriorityGet(NULL), &read_json_handle);
        if (ret != pdPASS)
            return ESP_FAIL;
        ret = xTaskCreate(logic_analyzer_draw_html, "draw", 4096, (void *)&ra, uxTaskPriorityGet(NULL), &draw_html_handle);
        if (ret != pdPASS)
            return ESP_FAIL;
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    if (ws_pkt.len)
    {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        if (ws_pkt.type == HTTPD_WS_TYPE_TEXT)
        {
            xQueueSend(read_json_queue, ws_pkt.payload, portMAX_DELAY);
        }
    }
    free(buf);
    return ret;
}
// uri handlers
static const httpd_uri_t la_gh = {
    .uri = "/la",
    .method = HTTP_GET,
    .handler = logic_analyzer_get_handler,
    .user_ctx = NULL};
static const httpd_uri_t la_ws = {
    .uri = "/la/ws",
    .method = HTTP_GET,
    .handler = logic_analyzer_ws_handler,
    .user_ctx = NULL,
    .is_websocket = true};

// register uri handlers on runing server
esp_err_t logic_analyzer_register_uri_handlers(httpd_handle_t server)
{
    esp_err_t ret = ESP_OK;
    ret = httpd_register_uri_handler(server, &la_ws);
    if (ret)
        goto _ret;
    ret = httpd_register_uri_handler(server, &la_gh);
    if (ret)
        goto _ret;
_ret:
    return ret;
}
