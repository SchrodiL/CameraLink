/*
 * msp_codec.h — MSP / MSPv2 framing: CRC, encoder and byte-at-a-time decoder.
 *
 * Wire format (all multi-byte integers little-endian):
 *
 *   MSPv1:         '$' 'M' <dir> size(1) cmd(1) [jumbo_size(2)] payload... xor(1)
 *                  checksum = XOR of (size, cmd, [jumbo], payload)
 *
 *   MSPv2 native:  '$' 'X' <dir> flags(1) cmd(2) size(2) payload... crc8(1)
 *                  crc8 = CRC8-DVB-S2 (poly 0xD5) over (flags, cmd, size, payload)
 *
 *   MSPv2 over v1: v1 shell with cmd == 255 (MSP_V2_FRAME_ID) wrapping a v2
 *                  frame; carries both the v2 crc8 and the outer v1 xor.
 *
 *   <dir>: '<' = request, '>' = reply, '!' = error reply.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Frame / protocol version. */
#define MSP_V1          0
#define MSP_V2_OVER_V1  1
#define MSP_V2_NATIVE   2

/* v1 command byte that signals "a v2 frame is embedded in this v1 payload". */
#define MSP_V2_FRAME_ID 255

/* Direction bytes. */
#define MSP_DIR_REQUEST  '<'   /* 0x3C  host -> FC  */
#define MSP_DIR_REPLY    '>'   /* 0x3E  FC -> host  */
#define MSP_DIR_ERROR    '!'   /* 0x21  FC -> host (error) */

/* Largest payload the codec will hold. Raise this if you need to read
 * large blobs (EEPROM, dataflash, OSD config). 512 covers all normal
 * telemetry including MSP_BOXNAMES (~307 bytes). */
#ifndef MSP_MAX_PAYLOAD
#define MSP_MAX_PAYLOAD 512
#endif

/* ------------------------------------------------------------------ */
/* A fully decoded packet: command + payload, header/checksum verified. */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t cmd;
    uint8_t  version;    /* MSP_V1 / MSP_V2_OVER_V1 / MSP_V2_NATIVE */
    uint8_t  direction;  /* MSP_DIR_REQUEST / REPLY / ERROR */
    uint8_t  flags;      /* MSPv2 flags byte (0 for v1) */
    uint16_t size;       /* payload length in bytes */
    uint8_t  payload[MSP_MAX_PAYLOAD];
} msp_packet_t;

/* ------------------------------------------------------------------ */
/* Checksums                                                           */
/* ------------------------------------------------------------------ */
uint8_t msp_crc8_dvb_s2(uint8_t crc, uint8_t a);
uint8_t msp_crc8_update(uint8_t crc, const uint8_t *data, size_t len);
uint8_t msp_xor_checksum(uint8_t sum, const uint8_t *data, size_t len);

/* ------------------------------------------------------------------ */
/* Encoder                                                             */
/* ------------------------------------------------------------------ */
/* Build one complete frame into dst. Returns bytes written, or 0 if the
 * frame does not fit in cap. `payload` may be NULL when size == 0. */
size_t msp_encode(uint8_t *dst, size_t cap, uint8_t version, uint8_t direction,
                  uint16_t cmd, uint8_t flags, const uint8_t *payload, uint16_t size);

/* ------------------------------------------------------------------ */
/* Incremental decoder (feed one byte at a time)                       */
/* ------------------------------------------------------------------ */
typedef enum {
    MSP_DECODE_NEED_MORE = 0,  /* keep feeding bytes */
    MSP_DECODE_DONE      = 1,  /* a packet was fully decoded into *out */
    MSP_DECODE_ERROR     = -1, /* bad checksum / malformed; decoder reset */
} msp_decode_result_t;

typedef struct {
    uint8_t  state;
    uint8_t  version;
    uint8_t  direction;
    uint16_t offset;        /* payload bytes accumulated so far */
    uint16_t data_size;     /* expected payload length */
    uint16_t cmd;
    uint8_t  flags;
    uint8_t  checksum1;     /* v1 XOR accumulator */
    uint8_t  checksum2;     /* v2 CRC8 accumulator */
    uint8_t  jumbo;         /* v1 size byte stash (for jumbo frames) */
    uint8_t  buf[MSP_MAX_PAYLOAD];
} msp_decoder_t;

void msp_decoder_init(msp_decoder_t *d);

/* Feed one byte. Returns MSP_DECODE_DONE when a complete, verified packet
 * is available in *out (may be NULL if the caller only wants the result). */
msp_decode_result_t msp_decoder_push(msp_decoder_t *d, uint8_t c, msp_packet_t *out);

#ifdef __cplusplus
}
#endif
