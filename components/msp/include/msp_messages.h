/*
 * msp_messages.h — typed helpers for common MSP messages.
 *
 * Header-only little-endian readers plus structs/decode/build helpers for
 * the messages the demo uses. Add your own alongside these.
 */
#pragma once

#include "msp_codec.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* --- little-endian readers over a payload --- */
static inline uint8_t  msp_rd_u8 (const uint8_t *p) { return p[0]; }
static inline uint16_t msp_rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline int16_t  msp_rd_i16(const uint8_t *p) { return (int16_t)msp_rd_u16(p); }
static inline uint32_t msp_rd_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline int32_t  msp_rd_i32(const uint8_t *p) { return (int32_t)msp_rd_u32(p); }

/* --- structs --- */
typedef struct {
    uint8_t protocol_version;
    uint8_t api_major;
    uint8_t api_minor;
} msp_api_version_t;

typedef struct {
    uint8_t year;   /* year - 2000 (not a semantic major version) */
    uint8_t month;
    uint8_t patch;
} msp_fc_version_t;

typedef struct {
    int16_t acc[3];    /* x,y,z — 512 == 1 g in Betaflight */
    int16_t gyro[3];   /* x,y,z — deg/s (scaled) */
    int16_t mag[3];    /* x,y,z */
} msp_raw_imu_t;

typedef struct {
    int16_t roll;      /* 0.1 deg */
    int16_t pitch;     /* 0.1 deg */
    int16_t yaw;       /* deg */
} msp_attitude_t;

typedef struct {
    uint8_t  vbat;         /* 0.1 V */
    uint16_t mah_drawn;    /* mAh */
    uint16_t rssi;         /* 0..1023 */
    int16_t  amperage;     /* 0.01 A (signed) */
    uint16_t voltage;      /* 0.01 V (0 on newer firmware) */
} msp_analog_t;

typedef struct {
    uint8_t  cell_count;
    uint16_t capacity_mah;
    uint8_t  voltage;      /* 0.1 V */
    uint16_t mah_drawn;
    int16_t  current;      /* 0.01 A (signed) */
    uint8_t  battery_state;
    uint16_t voltage2;     /* 0.01 V */
} msp_battery_state_t;

typedef struct {
    uint16_t cycle_time_us;
    uint16_t i2c_error_count;
    uint16_t sensor;           /* bitmask: 0x01 acc, 0x02 baro, 0x04 mag, 0x08 gps */
    uint32_t flight_mode_flags;
    uint8_t  pid_profile;
    uint16_t system_load_pct;
    uint16_t gyro_cycle_time_us;
} msp_status_t;

typedef struct {
    uint8_t  fix_type;      /* 0=none, 2=2D, 3=3D */
    uint8_t  num_sat;       /* 卫星数 */
    int32_t  lat;           /* 纬度，1e-7 度 */
    int32_t  lon;           /* 经度，1e-7 度 */
    uint16_t alt_m;         /* 海拔，米 */
    uint16_t speed_cms;     /* 地速，cm/s */
    uint16_t course_degx10; /* 航向，0.1 度 */
} msp_raw_gps_t;

/* --- decode helpers (return false if payload too short) --- */
static inline bool msp_decode_api_version(const msp_packet_t *p, msp_api_version_t *o) {
    if (p->size < 3) return false;
    o->protocol_version = msp_rd_u8(p->payload);
    o->api_major       = msp_rd_u8(p->payload + 1);
    o->api_minor       = msp_rd_u8(p->payload + 2);
    return true;
}

static inline bool msp_decode_fc_version(const msp_packet_t *p, msp_fc_version_t *o) {
    if (p->size < 3) return false;
    o->year  = msp_rd_u8(p->payload);
    o->month = msp_rd_u8(p->payload + 1);
    o->patch = msp_rd_u8(p->payload + 2);
    return true;
}

static inline bool msp_decode_raw_imu(const msp_packet_t *p, msp_raw_imu_t *o) {
    if (p->size < 18) return false;
    for (int i = 0; i < 3; i++) { o->acc[i]  = msp_rd_i16(p->payload + 0 + 2 * i); }
    for (int i = 0; i < 3; i++) { o->gyro[i] = msp_rd_i16(p->payload + 6 + 2 * i); }
    for (int i = 0; i < 3; i++) { o->mag[i]  = msp_rd_i16(p->payload + 12 + 2 * i); }
    return true;
}

static inline bool msp_decode_attitude(const msp_packet_t *p, msp_attitude_t *o) {
    if (p->size < 6) return false;
    o->roll  = msp_rd_i16(p->payload);
    o->pitch = msp_rd_i16(p->payload + 2);
    o->yaw   = msp_rd_i16(p->payload + 4);
    return true;
}

static inline bool msp_decode_analog(const msp_packet_t *p, msp_analog_t *o) {
    if (p->size < 7) return false;
    o->vbat      = msp_rd_u8 (p->payload);
    o->mah_drawn = msp_rd_u16(p->payload + 1);
    o->rssi      = msp_rd_u16(p->payload + 3);
    o->amperage  = msp_rd_i16(p->payload + 5);
    o->voltage   = (p->size >= 9) ? msp_rd_u16(p->payload + 7) : 0;
    return true;
}

static inline bool msp_decode_battery_state(const msp_packet_t *p, msp_battery_state_t *o) {
    if (p->size < 9) return false;
    o->cell_count    = msp_rd_u8 (p->payload);
    o->capacity_mah  = msp_rd_u16(p->payload + 1);
    o->voltage       = msp_rd_u8 (p->payload + 3);
    o->mah_drawn     = msp_rd_u16(p->payload + 4);
    o->current       = msp_rd_i16(p->payload + 6);
    o->battery_state = msp_rd_u8 (p->payload + 8);
    o->voltage2      = (p->size >= 11) ? msp_rd_u16(p->payload + 9) : 0;
    return true;
}

static inline bool msp_decode_status(const msp_packet_t *p, msp_status_t *o) {
    if (p->size < 11) return false;
    o->cycle_time_us      = msp_rd_u16(p->payload);
    o->i2c_error_count    = msp_rd_u16(p->payload + 2);
    o->sensor             = msp_rd_u16(p->payload + 4);
    o->flight_mode_flags  = msp_rd_u32(p->payload + 6);
    o->pid_profile        = msp_rd_u8 (p->payload + 10);
    o->system_load_pct    = (p->size >= 13) ? msp_rd_u16(p->payload + 11) : 0;
    o->gyro_cycle_time_us = (p->size >= 15) ? msp_rd_u16(p->payload + 13) : 0;
    return true;
}

static inline bool msp_decode_raw_gps(const msp_packet_t *p, msp_raw_gps_t *o) {
    if (p->size < 16) return false;
    o->fix_type      = msp_rd_u8 (p->payload);
    o->num_sat       = msp_rd_u8 (p->payload + 1);
    o->lat           = msp_rd_i32(p->payload + 2);
    o->lon           = msp_rd_i32(p->payload + 6);
    o->alt_m         = msp_rd_u16(p->payload + 10);
    o->speed_cms     = msp_rd_u16(p->payload + 12);
    o->course_degx10 = msp_rd_u16(p->payload + 14);
    return true;
}

/* Decode MSP_RC payload: 16 × uint16 channel values (1000..2000 us). */
static inline bool msp_decode_rc(const msp_packet_t *p, uint16_t ch[16]) {
    if (p->size < 32) return false;
    for (int i = 0; i < 16; i++) ch[i] = msp_rd_u16(p->payload + 2 * i);
    return true;
}

/* Copy a string-valued payload into dst (NUL terminated, truncated). */
static inline void msp_copy_string(const msp_packet_t *p, char *dst, size_t dst_size) {
    if (dst_size == 0) return;
    size_t n = p->size < (dst_size - 1) ? p->size : (dst_size - 1);
    memcpy(dst, p->payload, n);
    dst[n] = '\0';
}

/* --- build helpers for commands --- */

/* Build MSP_SET_RAW_RC payload (16 channels, 1000..2000 us). Returns length. */
static inline uint16_t msp_build_raw_rc(uint8_t *dst, const uint16_t ch[16]) {
    for (int i = 0; i < 16; i++) {
        dst[2 * i]     = (uint8_t)(ch[i] & 0xFF);
        dst[2 * i + 1] = (uint8_t)(ch[i] >> 8);
    }
    return 32;
}

/* Build MSP_SET_MOTOR payload (8 motors). Returns length. */
static inline uint16_t msp_build_motor(uint8_t *dst, const uint16_t motor[8]) {
    for (int i = 0; i < 8; i++) {
        dst[2 * i]     = (uint8_t)(motor[i] & 0xFF);
        dst[2 * i + 1] = (uint8_t)(motor[i] >> 8);
    }
    return 16;
}

/* --- MSP2_SET_TEXT build helpers --- */

/* Build MSP2_SET_TEXT payload: [text_type][length][chars...].
 * text is truncated to max_len bytes. Returns payload length. */
static inline uint16_t msp_build_set_text(uint8_t *dst, uint8_t text_type,
                                          const char *text, uint8_t max_len)
{
    size_t len = strlen(text);
    if (len > max_len) len = max_len;
    dst[0] = text_type;
    dst[1] = (uint8_t)len;
    memcpy(dst + 2, text, len);
    return (uint16_t)(2 + len);
}

/* Build MSP2_SET_TEXT payload for a custom OSD message (msg 0..3, max 16 chars). */
static inline uint16_t msp_build_custom_msg(uint8_t *dst, uint8_t msg_idx, const char *text)
{
    return msp_build_set_text(dst, (uint8_t)(MSP2TEXT_CUSTOM_MSG_0 + msg_idx), text,
                              OSD_CUSTOM_MSG_MAX_LEN);
}

/* mwosd / MAX7456 OSD 字体里的电池图标字形字节。
 * 对应 Betaflight osd_symbols.h 的 SYM_BATT_FULL..EMPTY / SYM_MAIN_BATT。
 * 0x90..0x96 是电量格图标（满→空），0x97 是主电池图标。 */
#define MSP_OSD_SYM_BATT_FULL   0x90
#define MSP_OSD_SYM_BATT_EMPTY  0x96
#define MSP_OSD_SYM_MAIN_BATT   0x97
#define MSP_OSD_SYM_REC         0x09   /* 录像(REC)图标 */

/* GPS / 速度 / 海拔图标字形字节。
 * 对应 Betaflight osd_symbols.h 的 SYM_SAT_L / SYM_SPEED / SYM_ALTITUDE。
 * 注意 0x70 落在可打印 ASCII 区间（原 'p' 位），sanitize 天然放行。 */
#define MSP_OSD_SYM_SAT         0x1E
#define MSP_OSD_SYM_SPEED       0x70
#define MSP_OSD_SYM_ALTITUDE    0x7F

/* 判断一个字节是否是 OSD 字体里可用的字形：可打印 ASCII 或显式允许的图标。 */
static inline bool msp_osd_sym_ok(unsigned char c)
{
    if (c >= 0x20 && c <= 0x7E) return true; /* 可打印 ASCII */
    if (c >= MSP_OSD_SYM_BATT_FULL && c <= MSP_OSD_SYM_MAIN_BATT) return true;
    return c == MSP_OSD_SYM_SAT || c == MSP_OSD_SYM_ALTITUDE || c == MSP_OSD_SYM_REC;
}

/* 把文本过滤成 OSD 字体能显示的字形。
 * MAX7456 / mwosd 字体只有 256 个字形：0x20..0x7E 是普通文本，
 * 0x00..0x1F 与 0x7F..0xFF 是特殊图标（电池/GPS/箭头等），
 * 其它（中文/非 ASCII，含 UTF-8 多字节）无法显示。
 * 不能显示的字符一律替换成空格，避免 OSD 出现乱码图标；唯一例外是
 * 本模块显式用到的图标字节（电池 0x90..0x97、卫星 0x1E、海拔 0x7F），
 * 见 msp_osd_sym_ok()。
 * 返回写入的字符数（不含结尾 NUL）。dst 需至少 max_len+1 字节。 */
static inline int msp_osd_sanitize(char *dst, const char *src, int max_len)
{
    int i = 0;
    for (; i < max_len && src[i] != '\0'; i++) {
        unsigned char c = (unsigned char)src[i];
        dst[i] = msp_osd_sym_ok(c) ? (char)c : ' ';
    }
    dst[i] = '\0';
    return i;
}
