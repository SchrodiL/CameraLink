/*
 * msp_uart.h — ESP-IDF UART transport for the MSP host layer.
 *
 * Wires the msp_host_t callbacks to a hardware UART. Configure pins/baud,
 * call msp_uart_init(), then msp_uart_bind().
 */
#pragma once

#include "msp_host.h"
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uart_port_t uart_num;   /* e.g. UART_NUM_1 (UART0 is usually the console) */
    int         tx_pin;     /* ESP TX -> FC RX */
    int         rx_pin;     /* ESP RX -> FC TX */
    int         baud_rate;  /* MSP default is 115200 */
} msp_uart_config_t;

/* Install the UART driver and configure it. Returns esp_err_t. */
esp_err_t msp_uart_init(const msp_uart_config_t *cfg);

/* Bind a host session to this UART using the given MSP version. */
void msp_uart_bind(msp_host_t *host, uint8_t msp_version);

/* Millisecond tick (provided for completeness / custom transports). */
uint32_t msp_uart_tick_ms(void);

/* Discard any bytes waiting in the RX ring buffer. Use after fire-and-forget
 * writes (msp_host_send) when the FC replies but the host never reads, so the
 * ring buffer does not slowly fill up. */
void msp_uart_flush_rx(void);

#ifdef __cplusplus
}
#endif
