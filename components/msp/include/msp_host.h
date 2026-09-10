/*
 * msp_host.h — transport-agnostic MSP client (host) session layer.
 *
 * Sends requests and collects/parses replies. The transport is abstracted
 * behind three callbacks so the same code works over UART, USB-CDC or any
 * byte stream. msp_uart.h provides a ready-made UART transport.
 */
#pragma once

#include "msp_codec.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Byte transport callbacks.
 *  - read():  block up to timeout_ms; return bytes read (0 = timeout), <0 = error.
 *  - write(): return bytes written, <0 = error.
 *  - tick_ms(): monotonic milliseconds (used for the request deadline). */
typedef int      (*msp_read_fn)(void *ctx, uint8_t *buf, int max_len, uint32_t timeout_ms);
typedef int      (*msp_write_fn)(void *ctx, const uint8_t *buf, int len);
typedef uint32_t (*msp_tick_fn)(void);

typedef struct {
    void           *ctx;
    msp_read_fn     read;
    msp_write_fn    write;
    msp_tick_fn     tick_ms;
    uint8_t         version;   /* MSP_V2_NATIVE (default) / MSP_V1 / MSP_V2_OVER_V1 */
    msp_decoder_t   dec;
} msp_host_t;

void msp_host_init(msp_host_t *h, msp_read_fn read, msp_write_fn write,
                   msp_tick_fn tick_ms, void *ctx, uint8_t version);

/* Send a request (fire-and-forget). Use for commands with no reply
 * (e.g. MSP_SET_RAW_RC, MSP_SET_MOTOR). Returns MSP_HOST_*. */
int msp_host_send(msp_host_t *h, uint16_t cmd, const uint8_t *payload, uint16_t size);

/* Send a request and block until the matching reply arrives or timeout.
 * On success (MSP_HOST_OK) *reply holds the decoded packet. */
int msp_host_request(msp_host_t *h, uint16_t cmd, const uint8_t *payload, uint16_t size,
                     msp_packet_t *reply, uint32_t timeout_ms);

/* Feed already-read bytes into the decoder (non-blocking). Returns the last
 * MSP_DECODE_* result; on MSP_DECODE_DONE *out holds the packet. */
msp_decode_result_t msp_host_feed(msp_host_t *h, const uint8_t *data, int len, msp_packet_t *out);

/* Return codes. */
#define MSP_HOST_OK           0
#define MSP_HOST_ERR_ENCODE  -1
#define MSP_HOST_ERR_WRITE   -2
#define MSP_HOST_ERR_TIMEOUT -3
#define MSP_HOST_ERR_READ    -4
#define MSP_HOST_ERR_BADCRC  -5
#define MSP_HOST_ERR_NOREPLY -6   /* FC answered with an '!' error reply */

#ifdef __cplusplus
}
#endif
