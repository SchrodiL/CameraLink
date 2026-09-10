/*
 * msp_host.c — transport-agnostic MSP client session.
 */
#include "msp_host.h"
#include <string.h>

void msp_host_init(msp_host_t *h, msp_read_fn read, msp_write_fn write,
                   msp_tick_fn tick_ms, void *ctx, uint8_t version)
{
    memset(h, 0, sizeof(*h));
    h->read = read;
    h->write = write;
    h->tick_ms = tick_ms;
    h->ctx = ctx;
    h->version = version;
    msp_decoder_init(&h->dec);
}

int msp_host_send(msp_host_t *h, uint16_t cmd, const uint8_t *payload, uint16_t size)
{
    uint8_t frame[MSP_MAX_PAYLOAD + 16];

    size_t n = msp_encode(frame, sizeof(frame), h->version, MSP_DIR_REQUEST,
                          cmd, 0, payload, size);
    if (n == 0) return MSP_HOST_ERR_ENCODE;

    int w = h->write(h->ctx, frame, (int)n);
    if (w != (int)n) return MSP_HOST_ERR_WRITE;
    return MSP_HOST_OK;
}

msp_decode_result_t msp_host_feed(msp_host_t *h, const uint8_t *data, int len, msp_packet_t *out)
{
    msp_decode_result_t r = MSP_DECODE_NEED_MORE;
    for (int i = 0; i < len; i++) {
        r = msp_decoder_push(&h->dec, data[i], out);
        if (r != MSP_DECODE_NEED_MORE) return r;
    }
    return r;
}

int msp_host_request(msp_host_t *h, uint16_t cmd, const uint8_t *payload, uint16_t size,
                     msp_packet_t *reply, uint32_t timeout_ms)
{
    int rc = msp_host_send(h, cmd, payload, size);
    if (rc != MSP_HOST_OK) return rc;

    uint32_t deadline = h->tick_ms() + timeout_ms;
    uint8_t buf[128];

    for (;;) {
        uint32_t now = h->tick_ms();
        if ((int32_t)(deadline - now) <= 0) return MSP_HOST_ERR_TIMEOUT;

        int r = h->read(h->ctx, buf, sizeof(buf), deadline - now);
        if (r < 0) return MSP_HOST_ERR_READ;
        if (r == 0) return MSP_HOST_ERR_TIMEOUT;

        for (int i = 0; i < r; i++) {
            msp_packet_t pkt;
            msp_decode_result_t dr = msp_decoder_push(&h->dec, buf[i], &pkt);
            if (dr == MSP_DECODE_ERROR) return MSP_HOST_ERR_BADCRC;
            if (dr == MSP_DECODE_DONE) {
                if (pkt.cmd == cmd) {
                    if (pkt.direction == MSP_DIR_ERROR) return MSP_HOST_ERR_NOREPLY;
                    if (reply) *reply = pkt;
                    return MSP_HOST_OK;
                }
                /* unsolicited / mismatched packet: ignore and keep waiting */
            }
        }
    }
}
