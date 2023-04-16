#pragma once

#include "logic_analizer_hal.h"

#define PULSEVIEW_MAX_SAMPLE_RATE 100000000


// definition test_sample
#define LEDC_OUTPUT_IO (18)  
#define GPIO_BLINK (19)

#define IN_PORT_1 (22)
#define IN_PORT_2 (23)


// define uart port - default port
#define SUMP_UART_PORT_NUM 0
#define SUMP_TEST_TXD 1
#define SUMP_TEST_RXD 3
#define SUMP_TEST_RTS (UART_PIN_NO_CHANGE)
#define SUMP_TEST_CTS (UART_PIN_NO_CHANGE)
#define SUMP_UART_BAUD_RATE 921600
#define UART_BUF_SIZE (256)

// data buff size
#define MAX_CAPTURE_SIZE (32764 * 2)
// count sample
#define MAX_SAMPLE_COUNT (MAX_CAPTURE_SIZE / 2)
// max sample clock HZ
#define MAX_SAMPLE_RATE 20000000

void sump_task(void *arg);
void sump_la_cb(uint16_t *buf, int cnt, int clk);
void send_err_blink(int on_time, int off_time, int repeat);
