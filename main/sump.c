/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "sump.h"

#define BLINK_PIN 2
QueueHandle_t blink_queue;
typedef struct blink
{
    int on_time;
    int off_time;
    int repeat;
} blink_t;

void blink_task(void *p);

// int trigger = -1;
int first_trigger_pin = 0;
// int trigger_values;
int first_trigger_val = 0;
int divider = 0;
int readCount = 0;
int delayCount = 0;

static void sump_write_data(uint8_t *buf, int len);
static void sump_writeByte(uint8_t byte);
static void sump_cmd_parser(uint8_t cmdByte);
static void sump_get_metadata();
static void sump_capture_and_send_samples();

logic_analizer_config_t la_cfg =
    {
        .pin = {LEDC_OUTPUT_IO, -1, 23, -1, GPIO_BLINK, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, LEDC_OUTPUT_IO},
        //.pin = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        .pin_trigger = -1,
        .trigger_edge = GPIO_INTR_POSEDGE,
        .number_of_samples = 10000,
        .sample_rate = 1250000,
        .meashure_timeout = 1000, // portMAX_DELAY,
        .logic_analizer_cb = sump_la_cb};

static void sump_capture_and_send_samples()
{
    la_cfg.number_of_samples = readCount;
    la_cfg.sample_rate = 100000000 / (divider + 1);
    if (first_trigger_pin >= 0)
    {
        la_cfg.pin_trigger = la_cfg.pin[first_trigger_pin];
        //    send_err_blink(50,50,first_trigger_pin);
    }
    else
    {
        la_cfg.pin_trigger = -1;
    }

    la_cfg.trigger_edge = first_trigger_val ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
    //        send_err_blink(50,50,divider);
    int err = start_logic_analizer(&la_cfg);
    if (err)
        send_err_blink(50, 50, 4);
}
void sump_la_cb(uint16_t *buf, int cnt, int clk)
{
    if (buf == NULL)
    {
        send_err_blink(50, 50, 3);
        return;
    }

    // sump_write_data((uint8_t*)buf, readCount*2);
    /*
    uint16_t *buf_out = buf;
    for (int i = 0; i < readCount; i += 2)
    {
        sump_write_data((uint8_t *)(buf_out + 1), 2);
        sump_write_data((uint8_t *)(buf_out), 2);
        buf_out += 2;
    }
    */
    // sigrok - data send on reverse order ????
        uint16_t *bufff = buf + readCount-1;
        for (int i = 0; i < readCount; i += 2)
        {
            sump_write_data((uint8_t *)(bufff-1), 2);
            sump_write_data((uint8_t *)(bufff), 2);
            bufff -=2;
        }
    
}

// define uart port - default port

#define SUMP_TEST_TXD 1
#define SUMP_TEST_RXD 3
#define SUMP_TEST_RTS (UART_PIN_NO_CHANGE)
#define SUMP_TEST_CTS (UART_PIN_NO_CHANGE)
#define SUMP_UART_PORT_NUM 0
#define SUMP_UART_BAUD_RATE 921600
// 115200

#define UART_BUF_SIZE (256)
#define SUMP_TASK_STACK_SIZE (CONFIG_EXAMPLE_TASK_STACK_SIZE)

static void sump_config_uart()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = SUMP_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = ESP_INTR_FLAG_IRAM;

    ESP_ERROR_CHECK(uart_driver_install(SUMP_UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(SUMP_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SUMP_UART_PORT_NUM, SUMP_TEST_TXD, SUMP_TEST_RXD, SUMP_TEST_RTS, SUMP_TEST_CTS));
}

static void sump_getCmd4(uint8_t *cmd)
{
    uart_read_bytes(SUMP_UART_PORT_NUM, cmd, 4, portMAX_DELAY);
}
static uint8_t sump_getCmd()
{
    uint8_t buf;
    uart_read_bytes(SUMP_UART_PORT_NUM, &buf, 1, portMAX_DELAY);
    return buf;
}
static void sump_write_data(uint8_t *buf, int len)
{
    uart_write_bytes(SUMP_UART_PORT_NUM, (const char *)buf, len);
}
static void sump_writeByte(uint8_t byte)
{
    uart_write_bytes(SUMP_UART_PORT_NUM, &byte, 1);
}

// loop read sump command // test only
void sump_task(void *arg)
{
    xTaskCreate(blink_task, "blink_task", 2048 * 4, NULL, 1, NULL);
    sump_config_uart();
    while (1)
    {
        uint8_t cmd = sump_getCmd();
        // sump_writeByte(cmd);
        sump_cmd_parser(cmd);
    }
}

// test only
// data buff size
#define MAX_CAPTURE_SIZE 32764 * 2
// count sample
#define MAX_SAMPLE_COUNT MAX_CAPTURE_SIZE / 2
// max sample clock HZ
#define MAX_SAMPLE_RATE 20000000
/*
 *  SUMP COMMAND DEFINITION
 */

/* XON/XOFF are not supported. */
#define SUMP_RESET 0x00
#define SUMP_ARM 0x01
#define SUMP_QUERY 0x02
#define SUMP_XON 0x11
#define SUMP_XOFF 0x13

/* mask & values used, config ignored. only stage0 supported */
#define SUMP_TRIGGER_MASK_CH_A 0xC0
#define SUMP_TRIGGER_MASK_CH_B 0xC4
#define SUMP_TRIGGER_MASK_CH_C 0xC8
#define SUMP_TRIGGER_MASK_CH_D 0xCC

#define SUMP_TRIGGER_VALUES_CH_A 0xC1
#define SUMP_TRIGGER_VALUES_CH_B 0xC5
#define SUMP_TRIGGER_VALUES_CH_C 0xC9
#define SUMP_TRIGGER_VALUES_CH_D 0xCD

#define SUMP_TRIGGER_CONFIG_CH_A 0xC2
#define SUMP_TRIGGER_CONFIG_CH_B 0xC6
#define SUMP_TRIGGER_CONFIG_CH_C 0xCA
#define SUMP_TRIGGER_CONFIG_CH_D 0xCE

/* Most flags are ignored. */
#define SUMP_SET_DIVIDER 0x80
#define SUMP_SET_READ_DELAY_COUNT 0x81
#define SUMP_SET_FLAGS 0x82
#define SUMP_SET_RLE 0x0100

/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST 0x03
#define SUMP_GET_METADATA 0x04

/*
 *   @brief main sump command loop
 *   @param cmdByte - data byte from uart
 */
static void sump_cmd_parser(uint8_t cmdByte)
{
   static  int trigger = 0;
   static  int trigger_values = 0;

    uint8_t cmd4[4]; // four cmd buff
    switch (cmdByte)
    {
    case SUMP_RESET:
        break;
    case SUMP_QUERY:
        sump_write_data((uint8_t *)"1ALS", 4);
        break;
    case SUMP_ARM:
        sump_capture_and_send_samples();
        break;
    case SUMP_TRIGGER_MASK_CH_A:
        sump_getCmd4(cmd4);
        trigger = cmd4[1] & 0xff;
        trigger <<= 8;
        trigger |= cmd4[0];
        first_trigger_pin = -1; // trigger not defined
        if (trigger)
        {
            for (int i = 0; i < 16; i++)
                if ((trigger >> i) & 0x1)
                {
                    first_trigger_pin = i; // only one trigger pin
                    break;
                }
        }
        break;
    case SUMP_TRIGGER_VALUES_CH_A:
        sump_getCmd4(cmd4);
        trigger_values = cmd4[1] & 0xff;
        trigger_values <<= 8;
        trigger_values |= cmd4[0];
        first_trigger_val = 0;
        if (trigger)
        {
            first_trigger_val = (trigger_values >> first_trigger_pin) & 1; // 0/1 value trigger
        }
        break;
    case SUMP_TRIGGER_MASK_CH_B:
    case SUMP_TRIGGER_MASK_CH_C:
    case SUMP_TRIGGER_MASK_CH_D:
    case SUMP_TRIGGER_VALUES_CH_B:
    case SUMP_TRIGGER_VALUES_CH_C:
    case SUMP_TRIGGER_VALUES_CH_D:
    case SUMP_TRIGGER_CONFIG_CH_A:
    case SUMP_TRIGGER_CONFIG_CH_B:
    case SUMP_TRIGGER_CONFIG_CH_C:
    case SUMP_TRIGGER_CONFIG_CH_D:
        sump_getCmd4(cmd4);
        break;
    case SUMP_SET_DIVIDER: // divider from freq ????
        sump_getCmd4(cmd4);
        divider = cmd4[2];
        divider = divider << 8;
        divider += cmd4[1];
        divider = divider << 8;
        divider += cmd4[0];
        break;
    case SUMP_SET_READ_DELAY_COUNT: // samples or bytes ??????
        sump_getCmd4(cmd4);
        readCount = 4 * (((cmd4[1] << 8) | cmd4[0]) + 1);
        if (readCount > MAX_SAMPLE_COUNT)
            readCount = MAX_SAMPLE_COUNT;
        delayCount = 4 * (((cmd4[3] << 8) | cmd4[2]) + 1);
        if (delayCount > MAX_SAMPLE_COUNT)
            delayCount = MAX_SAMPLE_COUNT;
        break;
    case SUMP_SET_FLAGS:
        sump_getCmd4(cmd4);
        break;
    case SUMP_GET_METADATA:
        sump_get_metadata();
        break;
    case SUMP_SELF_TEST:
        break;
    default:
        break;
    }
}

static void sump_get_metadata()
{
    /* device name */
    sump_writeByte((uint8_t)0x01);
    // OLS_Port.write("AGLAMv0");
    sump_write_data((uint8_t *)"ESP32", 6);
    /* firmware version */
    sump_writeByte((uint8_t)0x02);
    sump_write_data((uint8_t *)"0.00", 5);
    /* sample memory */
    sump_writeByte((uint8_t)0x21);
    uint32_t capture_size = MAX_CAPTURE_SIZE; // buff size bytes ??
    sump_writeByte((uint8_t)(capture_size >> 24) & 0xFF);
    sump_writeByte((uint8_t)(capture_size >> 16) & 0xFF);
    sump_writeByte((uint8_t)(capture_size >> 8) & 0xFF);
    sump_writeByte((uint8_t)(capture_size >> 0) & 0xFF);
    /* sample rate (20MHz) */
    uint32_t capture_speed = MAX_SAMPLE_RATE;
    sump_writeByte((uint8_t)0x23);
    sump_writeByte((uint8_t)(capture_speed >> 24) & 0xFF);
    sump_writeByte((uint8_t)(capture_speed >> 16) & 0xFF);
    sump_writeByte((uint8_t)(capture_speed >> 8) & 0xFF);
    sump_writeByte((uint8_t)(capture_speed >> 0) & 0xFF);
    /* number of probes */
    sump_writeByte((uint8_t)0x40);
    sump_writeByte((uint8_t)0x10); // 16
    /* protocol version (2) */
    sump_writeByte((uint8_t)0x41);
    sump_writeByte((uint8_t)0x02);
    /* end of data */
    sump_writeByte((uint8_t)0x00);
}

void blink_task(void *p)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BLINK_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE};
    blink_t blink_data;
    gpio_config(&cfg);
    blink_queue = xQueueCreate(10, sizeof(blink_t));
    gpio_set_level(BLINK_PIN, 0);
    while (1)
    {
        xQueueReceive(blink_queue, &blink_data, portMAX_DELAY);
        while (blink_data.repeat--)
        {
            gpio_set_level(BLINK_PIN, 1);
            vTaskDelay(blink_data.on_time);
            gpio_set_level(BLINK_PIN, 0);
            vTaskDelay(blink_data.off_time);
        };
    }
}

void send_err_blink(int on_time, int off_time, int repeat)
{
    blink_t on_off;
    on_off.on_time = on_time;
    on_off.off_time = off_time;
    on_off.repeat = repeat;
    xQueueSend(blink_queue, &on_off, portMAX_DELAY);
}
