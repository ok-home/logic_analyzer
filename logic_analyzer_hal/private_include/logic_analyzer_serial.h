#pragma once

#ifdef CONFIG_ANALYZER_USE_UART
#include "driver/uart.h"
#define BUF_SIZE (1024)
static void logic_analyzer_serial_init()
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
}
static void logic_analyzer_serial_write_bytes(const char *buf, size_t size)
{
    uart_write_bytes(CONFIG_ANALYZER_UART_PORT_NUMBER, buf, size);
}
static int logic_analyzer_serial_read_bytes(char *buf, size_t size)
{
    return uart_read_bytes(CONFIG_ANALYZER_UART_PORT_NUMBER, buf, size, portMAX_DELAY);
}
#endif

#ifdef CONFIG_ANALYZER_USE_USB_SERIAL_JTAG // test only
#include "driver/usb_serial_jtag.h"
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0)
// hack - esp idf issue #12628
#include "hal/usb_serial_jtag_ll.h"
// 
#endif
static void logic_analyzer_serial_init()
{
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = 2048,
        .tx_buffer_size = 64 // tx_buffer_size = usb tx packet size
    };
    
    //USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&usb_serial_jtag_config);
}
static void logic_analyzer_serial_write_bytes(const char *buf, size_t size)
{
    usb_serial_jtag_write_bytes(buf, size,portMAX_DELAY);
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0)
    // hack - esp idf issue #12628
    usb_serial_jtag_ll_txfifo_flush();
    //
#endif
}
static int logic_analyzer_serial_read_bytes(char *buf, size_t size)
{
    return usb_serial_jtag_read_bytes(buf, size, portMAX_DELAY);
}

#endif