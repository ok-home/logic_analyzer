#pragma once

// define uart port - default port
#define SUMP_UART_PORT_NUM 0
#define SUMP_TEST_TXD 1
#define SUMP_TEST_RXD 3
#define SUMP_TEST_RTS (UART_PIN_NO_CHANGE)
#define SUMP_TEST_CTS (UART_PIN_NO_CHANGE)
#define SUMP_UART_BAUD_RATE 921600
#define UART_BUF_SIZE (256)

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
//#define SUMP_SET_RLE 0x0100

/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST 0x03
#define SUMP_GET_METADATA 0x04

#ifdef __cplusplus
extern "C" {
#endif

void logic_analizer_sump_task(void *arg);

#ifdef __cplusplus
}
#endif
