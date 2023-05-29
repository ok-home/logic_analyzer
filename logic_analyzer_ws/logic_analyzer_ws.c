#include <esp_log.h>
#include <esp_http_server.h>
#include <freertos/queue.h>

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
    .meashure_timeout = LA_DEFAULT_TiMEOUT,
    .logic_analyzer_cb = NULL};
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

static void logic_analyzer_cb(uint16_t *sample_buf, int samples, int sample_rate)
{
    char jsonstr[64];
    esp_err_t ret = 0;
    httpd_ws_frame_t ws_pkt;

    if (samples)
    {
        sprintf(jsonstr, "{\"rowID\":\"%s%02d\",\"rowVal\":\"%d\"}", rowID[ROW_MSMP], 0, samples);
        ret = send_ws_string(jsonstr);
        sprintf(jsonstr, "{\"rowID\":\"%s%02d\",\"rowVal\":\"%d\"}", rowID[ROW_MCLK], 0, sample_rate);
        ret = send_ws_string(jsonstr);

        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.type = HTTPD_WS_TYPE_BINARY;
        ws_pkt.payload = (uint8_t *)sample_buf; // la cb buff
        ws_pkt.len = samples * 2;
        ESP_LOGI(TAG, "Start samples transfer %d", ws_pkt.len);
        send_ws_string("Start samples transfer");
        ret = httpd_ws_send_data(ra.hd, ra.fd, &ws_pkt);
        if (ret)
        {
            ESP_LOGE(TAG, "Samples transfer err %d", ret);
            send_ws_string("Samples transfer err");
        }
        ESP_LOGI(TAG, "Samples transfer done");
        send_ws_string("Samples transfer done");
    }
    else
    {
        ESP_LOGE(TAG, "Error - callback imeout deteсted");
        send_ws_string("Error - callback imeout deteсted");
    }
}
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
static esp_err_t draw_html_options(void)
{
    char jsonstr[128];
    esp_err_t ret = 0;
    for (int i = 0; i < sizeof(options) / sizeof(options[0]); i++)
    {
        sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s\",\"rowVal\":\"%ld\",\"rowLbl\":\"%s\"}",
                hdrID[HDR_OPTIONS], options[i].datalist, options[i].value, options[i].label);
        ret = send_ws_string(jsonstr);
    }
    return ret;
}
static esp_err_t draw_html_config(void)
{
    char jsonstr[256];
    esp_err_t ret = 0;
    for (int i = 0; i < MAX_PIN; i++)
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
            hdrID[HDR_CONFIG], rowID[ROW_SMP], 0, rowLbl[ROW_LBL_SAMPLE], rowType[ROW_NUMBER], MAX_SAMPLE_CNT, MIN_SAMPLE_CNT, la_cfg.number_of_samples, 100, 0, rowID[ROW_LST], DATALIST_SAMPLE, CHECK_CFG_DATA_EVENT);
    ret = send_ws_string(jsonstr);

    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowMax\":\"%d\",\"rowMin\":\"%d\",\"rowVal\":\"%d\",\"rowStep\":\"%d\",\"rowDis\":\"%d\",\"rowList\":\"%s%02d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_CONFIG], rowID[ROW_CLK], 0, rowLbl[ROW_LBL_CLOCK], rowType[ROW_NUMBER], MAX_CLK, MIN_CLK, la_cfg.sample_rate, 1000, 0, rowID[ROW_LST], DATALIST_CLK, CHECK_CFG_DATA_EVENT);
    ret = send_ws_string(jsonstr);
    sprintf(jsonstr, "{\"hdrID\":\"%s\",\"rowID\":\"%s%02d\",\"rowLbl\":\"%s\",\"rowType\":\"%s\",\"rowMax\":\"%d\",\"rowMin\":\"%d\",\"rowVal\":\"%d\",\"rowStep\":\"%d\",\"rowDis\":\"%d\",\"rowList\":\"%s%02d\",\"rowEvent\":\"%d\"}",
            hdrID[HDR_CONFIG], rowID[ROW_TIMEOUT], 0, rowLbl[ROW_LBL_TIMEOUT], rowType[ROW_NUMBER], 50, -1, 
            la_cfg.meashure_timeout >0 ? la_cfg.meashure_timeout/100:la_cfg.meashure_timeout,// TICK = 10 MSek
            1, 0, rowID[ROW_LST], DATALIST_TIMEOUT, CHECK_CFG_DATA_EVENT);
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
static void logic_analyzer_draw_html(void *arg)
{
    draw_html_datalist();
    draw_html_options();
    draw_html_config();
    draw_html_start();
    vTaskDelete(NULL);
}
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
        if (json_to_str_parm(json_string, name, val) == ESP_OK)
        {
            if (strncmp(rowID[ROW_PIN], name, 3) == 0)
            {
                int pin_numb = atoi(name + 3);
                int gpio = atoi(val);
                la_cfg.pin[pin_numb] = gpio;
            }
            else if (strncmp(rowID[ROW_TRG], name, 3) == 0)
            {
                la_cfg.pin_trigger = atoi(val);
            }
            else if (strncmp(rowID[ROW_EDG], name, 3) == 0)
            {
                la_cfg.trigger_edge = atoi(val);
            }
            else if (strncmp(rowID[ROW_SMP], name, 3) == 0)
            {
                la_cfg.number_of_samples = atoi(val);
            }
            else if (strncmp(rowID[ROW_CLK], name, 3) == 0)
            {
                la_cfg.sample_rate = atoi(val);
            }
            else if (strncmp(rowID[ROW_TIMEOUT], name, 3) == 0)
            {
                la_cfg.meashure_timeout = atoi(val)>0 ? atoi(val)*100 : atoi(val);
//                ESP_LOGI(TAG,"Timeout=%d",la_cfg.meashure_timeout);
            }
        }
        else
        {
            if (strncmp(END_CFG_MSG, json_string, 6) == 0)
            {
                la_cfg.logic_analyzer_cb = logic_analyzer_cb;
                ret = start_logic_analyzer(&la_cfg);
                if (ret)
                {
                    ESP_LOGE(TAG, "Start logic analyzer error %X", ret);
                    send_ws_string("Start logic analyzer error");
                }
                else
                {
                    ESP_LOGI(TAG, "Start logic analyzer OK");
                    send_ws_string("Start logic analyzer OK");
                }
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
// main ws handle read ws messaje & send to queue
static esp_err_t logic_analyzer_ws_handler(httpd_req_t *req)
{
    esp_err_t ret = 0;
    if (req->method == HTTP_GET)
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
