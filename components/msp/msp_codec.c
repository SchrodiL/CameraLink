/*
 * msp_codec.c — MSP / MSPv2 frame encoder and incremental decoder.
 *
 * Faithfully mirrors Betaflight's src/main/msp/msp_serial.c framing
 * (see the header for the exact wire layout), plus MSPv1 jumbo-frame
 * handling that Betaflight's own decoder omits.
 */
#include "msp_codec.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* Checksums                                                           */
/* ------------------------------------------------------------------ */

uint8_t msp_crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

uint8_t msp_crc8_update(uint8_t crc, const uint8_t *data, size_t len)
{
    while (len--) {
        crc = msp_crc8_dvb_s2(crc, *data++);
    }
    return crc;
}

uint8_t msp_xor_checksum(uint8_t sum, const uint8_t *data, size_t len)
{
    while (len--) {
        sum ^= *data++;
    }
    return sum;
}

/* ------------------------------------------------------------------ */
/* Encoder                                                             */
/* ------------------------------------------------------------------ */

size_t msp_encode(uint8_t *dst, size_t cap, uint8_t version, uint8_t direction,
                  uint16_t cmd, uint8_t flags, const uint8_t *payload, uint16_t size)
{
    uint8_t *p = dst;

#define EMIT(b)                                   \
    do {                                          \
        if ((size_t)(p - dst) >= cap) return 0;   \
        *p++ = (uint8_t)(b);                      \
    } while (0)

    if (version == MSP_V1) {
        EMIT('$'); EMIT('M'); EMIT(direction);
        uint8_t *crc_start = p;                    /* XOR covers from here */
        if (size >= 255) {                         /* jumbo frame */
            EMIT(255);                             /* size marker */
            EMIT((uint8_t)cmd);
            EMIT((uint8_t)(size & 0xFF));          /* true size, LE */
            EMIT((uint8_t)((size >> 8) & 0xFF));
        } else {
            EMIT((uint8_t)size);
            EMIT((uint8_t)cmd);
        }
        uint8_t crc = 0;
        for (uint8_t *q = crc_start; q < p; q++) crc ^= *q;
        for (uint16_t i = 0; i < size; i++) { crc ^= payload[i]; EMIT(payload[i]); }
        EMIT(crc);

    } else if (version == MSP_V2_NATIVE) {
        EMIT('$'); EMIT('X'); EMIT(direction);
        uint8_t *crc_start = p;                    /* CRC8 covers from here */
        EMIT(flags);
        EMIT((uint8_t)(cmd & 0xFF));
        EMIT((uint8_t)((cmd >> 8) & 0xFF));
        EMIT((uint8_t)(size & 0xFF));
        EMIT((uint8_t)((size >> 8) & 0xFF));
        uint8_t crc = 0;
        for (uint8_t *q = crc_start; q < p; q++) crc = msp_crc8_dvb_s2(crc, *q);
        for (uint16_t i = 0; i < size; i++) { crc = msp_crc8_dvb_s2(crc, payload[i]); EMIT(payload[i]); }
        EMIT(crc);

    } else {                                       /* MSP_V2_OVER_V1 */
        /* A v2 frame inside a v1 shell: v1 size = 5 (v2 hdr) + payload + 1 (crc8).
         * Jumbo v2-over-v1 is an obscure legacy path; reject it and let the
         * caller switch to MSP_V2_NATIVE instead. */
        if (size >= 249) return 0;
        uint16_t v1_payload = (uint16_t)(5 + size + 1);

        EMIT('$'); EMIT('M'); EMIT(direction);
        uint8_t *crc_start = p;                    /* outer v1 XOR from here */
        EMIT((uint8_t)v1_payload);
        EMIT(MSP_V2_FRAME_ID);                     /* v1 cmd == 255 */
        uint8_t *v2_hdr = p;                       /* inner v2 CRC8 from here */
        EMIT(flags);
        EMIT((uint8_t)(cmd & 0xFF));
        EMIT((uint8_t)((cmd >> 8) & 0xFF));
        EMIT((uint8_t)(size & 0xFF));
        EMIT((uint8_t)((size >> 8) & 0xFF));

        uint8_t crc8 = 0;
        for (uint8_t *q = v2_hdr; q < p; q++) crc8 = msp_crc8_dvb_s2(crc8, *q);
        for (uint16_t i = 0; i < size; i++) { crc8 = msp_crc8_dvb_s2(crc8, payload[i]); EMIT(payload[i]); }
        EMIT(crc8);

        uint8_t crc = 0;
        for (uint8_t *q = crc_start; q < p; q++) crc ^= *q;
        EMIT(crc);
    }

#undef EMIT
    return (size_t)(p - dst);
}

/* ------------------------------------------------------------------ */
/* Decoder                                                             */
/* ------------------------------------------------------------------ */

enum {
    S_START = 0,
    S_MAGIC,
    S_DIR,
    /* v1 */
    S_V1_SIZE,
    S_V1_CMD,
    S_V1_JUMBO_L,
    S_V1_JUMBO_H,
    S_V1_PAYLOAD,
    S_V1_CHECKSUM,
    /* v2 native */
    S_V2_FLAGS,
    S_V2_CMD_L,
    S_V2_CMD_H,
    S_V2_SIZE_L,
    S_V2_SIZE_H,
    S_V2_PAYLOAD,
    S_V2_CHECKSUM,
    /* v2 over v1 */
    S_V2O1_FLAGS,
    S_V2O1_CMD_L,
    S_V2O1_CMD_H,
    S_V2O1_SIZE_L,
    S_V2O1_SIZE_H,
    S_V2O1_PAYLOAD,
    S_V2O1_CRC,
    S_V2O1_XOR,
};

void msp_decoder_init(msp_decoder_t *d)
{
    memset(d, 0, sizeof(*d));
    d->state = S_START;
}

static void dec_reset(msp_decoder_t *d)
{
    d->state = S_START;
    d->offset = 0;
    d->checksum1 = 0;
    d->checksum2 = 0;
    d->data_size = 0;
    d->jumbo = 0;
}

static msp_decode_result_t dec_finish(msp_decoder_t *d, msp_packet_t *out)
{
    if (out) {
        out->cmd = d->cmd;
        out->version = d->version;
        out->direction = d->direction;
        out->flags = d->flags;
        out->size = (uint16_t)d->offset;
        memcpy(out->payload, d->buf, d->offset);
    }
    dec_reset(d);
    return MSP_DECODE_DONE;
}

msp_decode_result_t msp_decoder_push(msp_decoder_t *d, uint8_t c, msp_packet_t *out)
{
    switch (d->state) {

    case S_START:
        if (c == '$') d->state = S_MAGIC;
        break;

    case S_MAGIC:
        if (c == 'M') { d->version = MSP_V1; d->state = S_DIR; }
        else if (c == 'X') { d->version = MSP_V2_NATIVE; d->state = S_DIR; }
        else dec_reset(d);
        break;

    case S_DIR:
        if (c == '<') d->direction = MSP_DIR_REQUEST;
        else if (c == '>') d->direction = MSP_DIR_REPLY;
        else if (c == '!') d->direction = MSP_DIR_ERROR;
        else { dec_reset(d); break; }
        d->checksum1 = 0;
        d->checksum2 = 0;
        d->offset = 0;
        d->data_size = 0;
        d->jumbo = 0;
        d->state = (d->version == MSP_V1) ? S_V1_SIZE : S_V2_FLAGS;
        break;

    /* ---- MSP v1 ---- */
    case S_V1_SIZE:
        d->checksum1 ^= c;
        d->jumbo = c;                       /* stash the size byte */
        d->state = S_V1_CMD;
        break;

    case S_V1_CMD:
        d->checksum1 ^= c;
        d->cmd = c;                         /* v1 command is a single byte */
        if (d->jumbo == 255) {
            d->state = S_V1_JUMBO_L;        /* jumbo frame: 2-byte size follows */
        } else if (c == MSP_V2_FRAME_ID && d->jumbo >= 6) {
            d->version = MSP_V2_OVER_V1;    /* v2 frame embedded in v1 payload */
            d->state = S_V2O1_FLAGS;
        } else {
            d->data_size = d->jumbo;
            d->offset = 0;
            d->state = (d->data_size > 0) ? S_V1_PAYLOAD : S_V1_CHECKSUM;
        }
        break;

    case S_V1_JUMBO_L:
        d->checksum1 ^= c;
        d->data_size = c;
        d->state = S_V1_JUMBO_H;
        break;

    case S_V1_JUMBO_H:
        d->checksum1 ^= c;
        d->data_size |= (uint16_t)(c << 8);
        if (d->data_size > MSP_MAX_PAYLOAD) { dec_reset(d); return MSP_DECODE_ERROR; }
        if (d->cmd == MSP_V2_FRAME_ID) {
            d->version = MSP_V2_OVER_V1;
            d->state = S_V2O1_FLAGS;
        } else {
            d->offset = 0;
            d->state = (d->data_size > 0) ? S_V1_PAYLOAD : S_V1_CHECKSUM;
        }
        break;

    case S_V1_PAYLOAD:
        if (d->offset >= d->data_size || d->offset >= MSP_MAX_PAYLOAD) { dec_reset(d); break; }
        d->buf[d->offset++] = c;
        d->checksum1 ^= c;
        if (d->offset == d->data_size) d->state = S_V1_CHECKSUM;
        break;

    case S_V1_CHECKSUM:
        if (d->checksum1 == c) return dec_finish(d, out);
        dec_reset(d);
        return MSP_DECODE_ERROR;

    /* ---- MSP v2 native ---- */
    case S_V2_FLAGS:
        d->flags = c;
        d->checksum2 = msp_crc8_dvb_s2(0, c);
        d->state = S_V2_CMD_L;
        break;

    case S_V2_CMD_L:
        d->cmd = c;
        d->checksum2 = msp_crc8_dvb_s2(d->checksum2, c);
        d->state = S_V2_CMD_H;
        break;

    case S_V2_CMD_H:
        d->cmd |= (uint16_t)(c << 8);
        d->checksum2 = msp_crc8_dvb_s2(d->checksum2, c);
        d->state = S_V2_SIZE_L;
        break;

    case S_V2_SIZE_L:
        d->data_size = c;
        d->checksum2 = msp_crc8_dvb_s2(d->checksum2, c);
        d->state = S_V2_SIZE_H;
        break;

    case S_V2_SIZE_H:
        d->data_size |= (uint16_t)(c << 8);
        d->checksum2 = msp_crc8_dvb_s2(d->checksum2, c);
        if (d->data_size > MSP_MAX_PAYLOAD) { dec_reset(d); return MSP_DECODE_ERROR; }
        d->offset = 0;
        d->state = (d->data_size > 0) ? S_V2_PAYLOAD : S_V2_CHECKSUM;
        break;

    case S_V2_PAYLOAD:
        if (d->offset >= d->data_size || d->offset >= MSP_MAX_PAYLOAD) { dec_reset(d); break; }
        d->buf[d->offset++] = c;
        d->checksum2 = msp_crc8_dvb_s2(d->checksum2, c);
        if (d->offset == d->data_size) d->state = S_V2_CHECKSUM;
        break;

    case S_V2_CHECKSUM:
        if (d->checksum2 == c) return dec_finish(d, out);
        dec_reset(d);
        return MSP_DECODE_ERROR;

    /* ---- MSP v2 over v1 ---- */
    case S_V2O1_FLAGS:
        d->flags = c;
        d->checksum1 ^= c;
        d->checksum2 = msp_crc8_dvb_s2(0, c);
        d->state = S_V2O1_CMD_L;
        break;

    case S_V2O1_CMD_L:
        d->cmd = c;
        d->checksum1 ^= c;
        d->checksum2 = msp_crc8_dvb_s2(d->checksum2, c);
        d->state = S_V2O1_CMD_H;
        break;

    case S_V2O1_CMD_H:
        d->cmd |= (uint16_t)(c << 8);
        d->checksum1 ^= c;
        d->checksum2 = msp_crc8_dvb_s2(d->checksum2, c);
        d->state = S_V2O1_SIZE_L;
        break;

    case S_V2O1_SIZE_L:
        d->data_size = c;
        d->checksum1 ^= c;
        d->checksum2 = msp_crc8_dvb_s2(d->checksum2, c);
        d->state = S_V2O1_SIZE_H;
        break;

    case S_V2O1_SIZE_H:
        d->data_size |= (uint16_t)(c << 8);
        d->checksum1 ^= c;
        d->checksum2 = msp_crc8_dvb_s2(d->checksum2, c);
        if (d->data_size > MSP_MAX_PAYLOAD) { dec_reset(d); return MSP_DECODE_ERROR; }
        d->offset = 0;
        d->state = (d->data_size > 0) ? S_V2O1_PAYLOAD : S_V2O1_CRC;
        break;

    case S_V2O1_PAYLOAD:
        if (d->offset >= d->data_size || d->offset >= MSP_MAX_PAYLOAD) { dec_reset(d); break; }
        d->buf[d->offset++] = c;
        d->checksum1 ^= c;
        d->checksum2 = msp_crc8_dvb_s2(d->checksum2, c);
        if (d->offset == d->data_size) d->state = S_V2O1_CRC;
        break;

    case S_V2O1_CRC:
        d->checksum1 ^= c;                  /* inner v2 crc is part of v1 payload */
        if (d->checksum2 == c) d->state = S_V2O1_XOR;
        else { dec_reset(d); return MSP_DECODE_ERROR; }
        break;

    case S_V2O1_XOR:
        if (d->checksum1 == c) return dec_finish(d, out);
        dec_reset(d);
        return MSP_DECODE_ERROR;

    default:
        dec_reset(d);
        break;
    }

    return MSP_DECODE_NEED_MORE;
}
