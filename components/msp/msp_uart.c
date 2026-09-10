/*
 * msp_uart.c — ESP-IDF UART transport for the MSP host.
 */
#include "msp_uart.h"
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static uart_port_t s_uart_port = UART_NUM_1;

uint32_t msp_uart_tick_ms(void)
{
    return esp_log_timestamp();
}

static int uart_read_cb(void *ctx, uint8_t *buf, int max_len, uint32_t timeout_ms)
{
    (void)ctx;
    return uart_read_bytes(s_uart_port, buf, (uint32_t)max_len, pdMS_TO_TICKS(timeout_ms));
}

static int uart_write_cb(void *ctx, const uint8_t *buf, int len)
{
    (void)ctx;
    return uart_write_bytes(s_uart_port, buf, (size_t)len);
}

esp_err_t msp_uart_init(const msp_uart_config_t *cfg)
{
    s_uart_port = cfg->uart_num;

    uart_config_t uc = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    esp_err_t err = uart_driver_install(s_uart_port, 2048, 2048, 0, NULL, 0);
    if (err != ESP_OK) return err;

    err = uart_param_config(s_uart_port, &uc);
    if (err != ESP_OK) return err;

    return uart_set_pin(s_uart_port, cfg->tx_pin, cfg->rx_pin,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void msp_uart_bind(msp_host_t *host, uint8_t msp_version)
{
    msp_host_init(host, uart_read_cb, uart_write_cb, msp_uart_tick_ms, NULL, msp_version);
}

void msp_uart_flush_rx(void)
{
    uart_flush_input(s_uart_port);
}
