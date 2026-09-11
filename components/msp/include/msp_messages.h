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
    int16_t acc[3];    /* x,y,z — 512 == 1 g（MSP 约定） */
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
    uint16_t alt_m;         /* 海拔（MSL），米 */
    uint16_t speed_cms;     /* 地速，cm/s */
    uint16_t course_degx10; /* 航向，0.1 度 */
    /* PDOP ×0.01。**可选字段**：Betaflight 自 MSP API 1.44 起才在 MSP_RAW_GPS
     * 末尾追加它，更早的固件只发前 16 字节。为 0 表示飞控没给（按星数估算）。 */
    uint16_t pdop_x100;
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
    /* PDOP 是可选尾巴：只有 MSP API >= 1.44 的固件才发，老固件到 14 字节就结束。
     * 所以这里按实际长度判断，不能无条件读——越界读到的是帧尾/垃圾。 */
    o->pdop_x100     = (p->size >= 18) ? msp_rd_u16(p->payload + 16) : 0;
    return true;
}

/*
 * MSP_SET_RAW_GPS 的载荷（14 字节）—— 把本机的 GPS 喂给飞控。
 *
 * 字段顺序与单位来自 Betaflight 的 msp.c：
 *   fix(1) numSat(1) lat(4) lon(4) alt(2) speed(2)
 * 注意 **alt 的单位是「米」**（飞控读到后自己 ×100 转 cm），speed 是 cm/s。
 * 飞控侧需要配成 gps_provider = MSP 才会采用；此时 gps.c 的 GPS_MSP 分支会
 * 把数据直接送进 onGpsNewData()，即当作真正的 GPS 用，不只是显示。
 */
static inline uint16_t msp_build_set_raw_gps(uint8_t *dst,
                                             uint8_t  fix_type,
                                             uint8_t  num_sat,
                                             int32_t  lat_1e7,
                                             int32_t  lon_1e7,
                                             uint16_t alt_m,
                                             uint16_t speed_cms)
{
    dst[0]  = fix_type;
    dst[1]  = num_sat;
    dst[2]  = (uint8_t)(lat_1e7);
    dst[3]  = (uint8_t)(lat_1e7 >> 8);
    dst[4]  = (uint8_t)(lat_1e7 >> 16);
    dst[5]  = (uint8_t)(lat_1e7 >> 24);
    dst[6]  = (uint8_t)(lon_1e7);
    dst[7]  = (uint8_t)(lon_1e7 >> 8);
    dst[8]  = (uint8_t)(lon_1e7 >> 16);
    dst[9]  = (uint8_t)(lon_1e7 >> 24);
    dst[10] = (uint8_t)(alt_m);
    dst[11] = (uint8_t)(alt_m >> 8);
    dst[12] = (uint8_t)(speed_cms);
    dst[13] = (uint8_t)(speed_cms >> 8);
    return 14;
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

/* MAX7456 类 OSD 字体的电池图标字形字节（各家 MSP OSD 实现通用的字形表）。
 * 0x90..0x96 是电量格图标（满→空），0x97 是主电池图标。 */
#define MSP_OSD_SYM_BATT_FULL   0x90
#define MSP_OSD_SYM_BATT_EMPTY  0x96
#define MSP_OSD_SYM_MAIN_BATT   0x97
#define MSP_OSD_SYM_REC         0x09   /* 录像(REC)图标 */

/* 经纬度标志字形。 */
#define MSP_OSD_SYM_LAT         0x89
#define MSP_OSD_SYM_LON         0x98

/* GPS / 速度 / 海拔图标字形字节。
 * 注意 0x70 落在可打印 ASCII 区间（原 'p' 位），sanitize 天然放行。 */
#define MSP_OSD_SYM_SAT         0x1E
#define MSP_OSD_SYM_SPEED       0x70
#define MSP_OSD_SYM_ALTITUDE    0x7F

/* 判断一个字节是否是 OSD 字体里可用的字形：可打印 ASCII 或显式允许的图标。 */
static inline bool msp_osd_sym_ok(unsigned char c)
{
    if (c >= 0x20 && c <= 0x7E) return true; /* 可打印 ASCII */
    if (c >= MSP_OSD_SYM_BATT_FULL && c <= MSP_OSD_SYM_MAIN_BATT) return true;
    return c == MSP_OSD_SYM_SAT || c == MSP_OSD_SYM_ALTITUDE || c == MSP_OSD_SYM_REC ||
           c == MSP_OSD_SYM_LAT || c == MSP_OSD_SYM_LON;
}

/* 把文本过滤成 OSD 字体能显示的字形。
 * MAX7456 类 OSD 字体只有 256 个字形：0x20..0x7E 是普通文本，
 * 0x00..0x1F 与 0x7F..0xFF 是特殊图标（电池/GPS/箭头等），
 * 其它（中文/非 ASCII，含 UTF-8 多字节）无法显示。
 *
 * **小写字母一律转成大写**：该字体的 0x61..0x7A 区间并非完整的字母表，
 * 部分字形被替换成了符号——例如 'f' 是箭头、'm' 也是箭头。曾经因为
 * "%um" 里的单位用了小写 m，OSD 上「8m」显示成了「8↗」。在唯一汇聚点
 * 统一转换，比要求每个调用方自己记得转大写可靠得多。
 *
 * 不能显示的字符一律替换成空格，避免 OSD 出现乱码图标；唯一例外是
 * 本模块显式用到的图标字节（电池 0x90..0x97、卫星 0x1E、海拔 0x7F、
 * 经纬度 0x89/0x98），见 msp_osd_sym_ok()。
 * 返回写入的字符数（不含结尾 NUL）。dst 需至少 max_len+1 字节。 */
static inline int msp_osd_sanitize(char *dst, const char *src, int max_len)
{
    int i = 0;
    for (; i < max_len && src[i] != '\0'; i++) {
        unsigned char c = (unsigned char)src[i];
        if (c >= 'a' && c <= 'z') {
            c = (unsigned char)(c - 'a' + 'A');
        }
        dst[i] = msp_osd_sym_ok(c) ? (char)c : ' ';
    }
    dst[i] = '\0';
    return i;
}
