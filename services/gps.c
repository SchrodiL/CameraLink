/* SPDX-License-Identifier: MIT
 * Copyright (c) 2025 SZ DJI Technology Co., Ltd.
 * Source: dji-sdk/Osmo-GPS-Controller-Demo (MIT)
 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gps.h"

#define TAG "LOGIC_GPS"

// Initialize GPS data structure
// 初始化 GPS 数据结构
static GPS_Data_t GPS_Data;

// Counter for consecutive invalid GPS readings
// GPS连续无效次数计数器
static uint8_t gps_invalid_count = 0;

// GPS 模块是否在发送 UBX 数据（收到任意 NAV-PVT 即置位，表示链路已建立；
// 与是否有定位 fix 无关，用于区分「已连接但 0 颗星」与「未连接 NO GPS」）。
static volatile bool s_gps_connected = false;

/**
 * @brief Initialize GPS data structure
 *        初始化 GPS 数据结构
 * 
 * Reset all fields in GPS data structure to initial values.
 * 将 GPS 数据结构的所有字段重置为初始值。
 */
static void init_gps_data(void) {
    GPS_Data.Year = 0;
    GPS_Data.Month = 0;
    GPS_Data.Day = 0;
    GPS_Data.Hour = 0;
    GPS_Data.Minute = 0;
    GPS_Data.Second = 0.0;

    GPS_Data.Latitude = 0.0;
    GPS_Data.Lat_Indicator = 'N';
    GPS_Data.Longitude = 0.0;
    GPS_Data.Lon_Indicator = 'E';

    GPS_Data.Speed_knots = 0.0;
    GPS_Data.Course = 0.0;
    GPS_Data.Altitude = 0.0;
    GPS_Data.Num_Satellites = 0;

    GPS_Data.Velocity_North = 0.0;
    GPS_Data.Velocity_East = 0.0;
    GPS_Data.Velocity_Descend = 0.0;

    // GPS_Data.Status = 0;
    GPS_Data.RMC_Valid = 0;
    GPS_Data.GGA_Valid = 0;
    GPS_Data.RMC_Latitude = 0.0;
    GPS_Data.RMC_Longitude = 0.0;
    GPS_Data.GGA_Latitude = 0.0;
    GPS_Data.GGA_Longitude = 0.0;
}

/**
 * @brief Check if GPS signal is found
 *        检查 GPS 是否已找到信号
 * 
 * @return bool Returns true if consecutive invalid count is less than 10, false otherwise
 *              如果 GPS 连续无效次数小于10，返回 true；否则返回 false
 */
bool is_gps_found(void) {
    return (gps_invalid_count < 10);
}

/**
 * @brief 检查 GPS 模块是否在发送 UBX 数据（连接已建立）
 *        Check if the GPS module is streaming UBX data (link established)
 *
 * @return bool Returns true if any NAV-PVT has been received, false otherwise
 *              收到过任意 NAV-PVT 返回 true，否则 false
 */
bool is_gps_connected(void) {
    return s_gps_connected;
}

/**
 * @brief Check if current GPS data is valid
 *        检查当前 GPS 数据是否有效
 * 
 * @return bool Returns true if GPS status is valid, false otherwise
 *              如果 GPS 状态为有效，返回 true；否则返回 false
 */
bool is_current_gps_data_valid(void) {
    if (GPS_Data.Status == 1) {
        return true;
    }
    return false;
}

const GPS_Data_t *gps_logic_get_data(void) {
    return &GPS_Data;
}

/* 数据就绪回调（供 DJI 后端挂接）。 */
static gps_data_ready_cb_t s_data_ready_cb = NULL;

void gps_set_data_ready_cb(gps_data_ready_cb_t cb) {
    s_data_ready_cb = cb;
}

/* ==================================================================== */
/* UBX 协议支持（u-blox M8/M9/M10）                                       */
/* ==================================================================== */

#define UBX_SYNC1      0xB5
#define UBX_SYNC2      0x62
#define UBX_CLASS_NAV  0x01
#define UBX_ID_PVT     0x07
#define UBX_CLASS_ACK  0x05
#define UBX_ID_ACK_NAK 0x00
#define UBX_ID_ACK_ACK 0x01
#define UBX_CLASS_CFG  0x06
#define UBX_CFG_PRT    0x00
#define UBX_CFG_MSG    0x01
#define UBX_CFG_RATE   0x08
#define UBX_CFG_GNSS   0x3E
#define UBX_CLASS_MON  0x0A
#define UBX_ID_MON_VER 0x04
#define UBX_CFG_VALSET 0x8A
#define UBX_VAL_LAYER_RAM 0x01   /* layer 必须是有效位掩码：0x01=RAM,0x02=BBR,0x04=Flash；0 会被 M9/M10 拒绝 */

/* VALSET key（对齐 BF 的 ubxValGetSetBytes_e；key 最高字节编码 value 长度） */
#define CFG_RATE_MEAS                0x30210001  // U2
#define CFG_RATE_NAV                 0x30210002  // U2
#define CFG_RATE_TIMEREF             0x20210003  // E1
#define CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007  // U1
#define CFG_MSGOUT_NMEA_ID_GGA_UART1 0x209100bb  // U1
#define CFG_MSGOUT_NMEA_ID_VTG_UART1 0x209100b1  // U1
#define CFG_MSGOUT_NMEA_ID_GSV_UART1 0x209100c5  // U1
#define CFG_MSGOUT_NMEA_ID_GLL_UART1 0x209100ca  // U1
#define CFG_MSGOUT_NMEA_ID_GSA_UART1 0x209100c0  // U1
#define CFG_MSGOUT_NMEA_ID_RMC_UART1 0x209100ac  // U1

/* 小端读取辅助函数 */
static uint16_t ubx_le16(const uint8_t *p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static int32_t ubx_le32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/* Fletcher-8 校验（UBX 帧校验覆盖 class/id/len/payload 字节） */
static void ubx_calc_checksum(const uint8_t *buf, size_t len, uint8_t *ck_a, uint8_t *ck_b) {
    uint8_t a = 0, b = 0;
    for (size_t i = 0; i < len; i++) {
        a = (uint8_t)(a + buf[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

/* 构建并发送一条 UBX 帧 */
static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    uint8_t frame[512];
    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = (uint8_t)(len & 0xFF);
    frame[5] = (uint8_t)(len >> 8);
    if (len > 0 && payload != NULL) {
        memcpy(&frame[6], payload, len);
    }
    uint8_t ck_a, ck_b;
    ubx_calc_checksum(&frame[2], (size_t)len + 4, &ck_a, &ck_b);
    frame[6 + len] = ck_a;
    frame[7 + len] = ck_b;
    uart_write_bytes(UART_GPS_PORT, (const char *)frame, (size_t)len + 8);
}

/* 发送 PUBX 配置命令（BF 用它切波特率：$PUBX,41,1,0003,0001,<baud>,0*XX\r\n）
 * PUBX 是 NMEA 风格配置命令，u-blox 在任意波特率下都能接收。 */
static void ubx_send_pubx_baud(int baud) {
    char cmd[64];
    int n = snprintf(cmd, sizeof(cmd), "$PUBX,41,1,0003,0001,%d,0", baud);
    uint8_t ck = 0;
    for (int i = 1; i < n; i++) {  // '$' 之后到结尾的异或
        ck ^= (uint8_t)cmd[i];
    }
    char full[80];
    int m = snprintf(full, sizeof(full), "%s*%02X\r\n", cmd, ck);
    uart_write_bytes(UART_GPS_PORT, full, (size_t)m);
}

/* VALSET 帧辅助：追加一个 key-value 对（key 4 字节小端 + value），返回写入字节数 */
static size_t ubx_valset_add(uint8_t *cfg, size_t offset, uint32_t key, const uint8_t *val, size_t val_len) {
    cfg[offset + 0] = (uint8_t)(key & 0xFF);
    cfg[offset + 1] = (uint8_t)((key >> 8) & 0xFF);
    cfg[offset + 2] = (uint8_t)((key >> 16) & 0xFF);
    cfg[offset + 3] = (uint8_t)((key >> 24) & 0xFF);
    for (size_t i = 0; i < val_len; i++) {
        cfg[offset + 4 + i] = val[i];
    }
    return 4 + val_len;
}

/* 发送 CFG-VALSET 帧（version + layer + reserved + cfgData） */
static void ubx_send_valset(const uint8_t *cfg, size_t cfg_len) {
    uint8_t payload[128];
    payload[0] = 0;                 // version
    payload[1] = UBX_VAL_LAYER_RAM; // layer
    payload[2] = 0;                 // reserved
    payload[3] = 0;                 // reserved
    memcpy(&payload[4], cfg, cfg_len);
    ubx_send(UBX_CLASS_CFG, UBX_CFG_VALSET, payload, 4 + cfg_len);
}

/* VALSET 配置（M9+）：10Hz + 使能 NAV-PVT + 禁 NMEA 输出（对齐 BF） */
static void ubx_config_valset(void) {
    uint8_t cfg[64];
    size_t off = 0;

    /* CFG-RATE：measRate=100ms(10Hz), navRate=1, timeRef=GPS */
    uint8_t meas[2] = {100, 0};
    uint8_t nav[2] = {1, 0};
    uint8_t timeref[1] = {1};  // GPS
    off += ubx_valset_add(cfg, off, CFG_RATE_MEAS, meas, 2);
    off += ubx_valset_add(cfg, off, CFG_RATE_NAV, nav, 2);
    off += ubx_valset_add(cfg, off, CFG_RATE_TIMEREF, timeref, 1);

    /* 使能 NAV-PVT（UART1） */
    uint8_t rate[1] = {1};
    off += ubx_valset_add(cfg, off, CFG_MSGOUT_UBX_NAV_PVT_UART1, rate, 1);

    /* 禁 NMEA 输出（GGA/VTG/GSV/GLL/GSA/RMC，UART1） */
    uint8_t zero[1] = {0};
    off += ubx_valset_add(cfg, off, CFG_MSGOUT_NMEA_ID_GGA_UART1, zero, 1);
    off += ubx_valset_add(cfg, off, CFG_MSGOUT_NMEA_ID_VTG_UART1, zero, 1);
    off += ubx_valset_add(cfg, off, CFG_MSGOUT_NMEA_ID_GSV_UART1, zero, 1);
    off += ubx_valset_add(cfg, off, CFG_MSGOUT_NMEA_ID_GLL_UART1, zero, 1);
    off += ubx_valset_add(cfg, off, CFG_MSGOUT_NMEA_ID_GSA_UART1, zero, 1);
    off += ubx_valset_add(cfg, off, CFG_MSGOUT_NMEA_ID_RMC_UART1, zero, 1);

    ubx_send_valset(cfg, off);
}

/* 调试：波特率探测失败时，dump 一段原始字节，帮助判断接线/波特率问题 */
static void gps_dump_raw_bytes(void) {
    uint8_t buf[160];
    int total = 0;
    // 在回退波特率下读 ~1s
    for (int t = 0; t < 50 && total < (int)sizeof(buf); t++) {
        int n = uart_read_bytes(UART_GPS_PORT, buf + total, sizeof(buf) - total, 20 / portTICK_PERIOD_MS);
        if (n > 0) total += n;
    }

    if (total == 0) {
        ESP_LOGW(TAG, "GPS: no bytes received at %d baud (check VCC/GND and TX/RX wiring)",
                 UART_GPS_BAUD_RATE);
        return;
    }

    ESP_LOGI(TAG, "GPS: %d raw bytes at %d baud:", total, UART_GPS_BAUD_RATE);
    ESP_LOG_BUFFER_HEX(TAG, buf, total);

    // 可打印字符转成字符串打印，便于直接读出 NMEA（$ 开头）或二进制 UBX
    char txt[161];
    int m = 0;
    for (int i = 0; i < total && m < (int)sizeof(txt) - 1; i++) {
        char c = (char)buf[i];
        txt[m++] = (c >= 0x20 && c < 0x7f) ? c : '.';
    }
    txt[m] = '\0';
    ESP_LOGI(TAG, "GPS raw: %s", txt);
}

/* 前向声明：gps_autobaud 位于 UBX 解析器（ubx_feed）之前，需要先声明 */
static bool ubx_feed(uint8_t b);
static void ubx_state_reset(void);
static volatile bool s_ubx_mon_ver_got = false;  // 波特率探测：收到 MON-VER 响应
static volatile bool s_ubx_ack_ack = false;      // 配置确认：收到 ACK-ACK
static volatile uint8_t s_ubx_platform = 0;      // 模块版本：8=M8, 9=M9, 10=M10（0=未知）

/* 探测 GPS 模块的实际波特率（BF 式：主动发 MON-VER poll，等模块响应）。
 * 返回探测到的波特率；失败时回退到 UART_GPS_BAUD_RATE。 */
static int gps_autobaud(void) {
    static const int candidates[] = {
        115200, 9600, 38400, 19200, 57600, 230400, 460800, 921600
    };

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        int baud = candidates[i];
        uart_set_baudrate(UART_GPS_PORT, baud);
        uart_flush_input(UART_GPS_PORT);

        s_ubx_mon_ver_got = false;
        /* 每个候选波特率开始前复位解析状态机，避免上个波特率的垃圾字节污染 */
        ubx_state_reset();
        /* 主动发 MON-VER poll；波特率正确时模块会回 MON-VER 响应 */
        ubx_send(UBX_CLASS_MON, UBX_ID_MON_VER, NULL, 0);

        /* MON-VER 响应约 200 字节以上（swVersion30+hwVersion10+extension300），
         * 必须按块读取，1 字节/次 × 40 次永远读不完，导致探测失败。 */
        uint8_t buf[64];
        for (int t = 0; t < 40; t++) {  // 最多约 400ms
            int n = uart_read_bytes(UART_GPS_PORT, buf, sizeof(buf), 10 / portTICK_PERIOD_MS);
            if (n > 0) {
                for (int k = 0; k < n; k++) {
                    ubx_feed(buf[k]);
                }
            }
            if (s_ubx_mon_ver_got) {
                ESP_LOGI(TAG, "GPS autobaud: detected %d", baud);
                uart_flush_input(UART_GPS_PORT);
                /* BF 式：检测到后发 PUBX 把模块切到目标波特率 + UBX-only，UART 同步切换 */
                if (baud != UART_GPS_BAUD_RATE) {
                    ubx_send_pubx_baud(UART_GPS_BAUD_RATE);
                    vTaskDelay(pdMS_TO_TICKS(100));  // 等模块完成波特率切换
                    uart_set_baudrate(UART_GPS_PORT, UART_GPS_BAUD_RATE);
                    uart_flush_input(UART_GPS_PORT);
                }
                return UART_GPS_BAUD_RATE;
            }
        }
    }

    ESP_LOGW(TAG, "GPS autobaud failed, falling back to %d", UART_GPS_BAUD_RATE);
    uart_set_baudrate(UART_GPS_PORT, UART_GPS_BAUD_RATE);
    uart_flush_input(UART_GPS_PORT);
    gps_dump_raw_bytes();
    return UART_GPS_BAUD_RATE;
}

/* 配置 GNSS 星座：默认开北斗，同时开 GPS/GLONASS/Galileo/QZSS/SBAS，越多越好。
 * 参考 Betaflight 的 GPS 自动配置（UBX-CFG-GNSS）。
 * numTrkChHw 只读（写 0）；numTrkChUse=0 表示用最大可用通道。 */
static void ubx_config_gnss(void) {
    uint8_t p[4 + 6 * 8] = {
        0x00,  // msgVer
        0x00,  // numTrkChHw（只读，写 0 即可）
        0x00,  // numTrkChUse（0 = 使用最大可用通道）
        0x06,  // numConfigBlocks = 6
        /* 每块 8 字节：gnssId, resTrkCh(保留), maxTrkCh, reserved, flags(4 字节, bit0=enable) */
        0x00, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00,  // GPS
        0x03, 0x00, 0x0E, 0x00, 0x01, 0x00, 0x00, 0x00,  // 北斗 BeiDou
        0x06, 0x00, 0x0E, 0x00, 0x01, 0x00, 0x00, 0x00,  // GLONASS
        0x02, 0x00, 0x0E, 0x00, 0x01, 0x00, 0x00, 0x00,  // Galileo
        0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00,  // QZSS
        0x01, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00,  // SBAS
    };
    ubx_send(UBX_CLASS_CFG, UBX_CFG_GNSS, p, sizeof(p));
}

/* 配置 u-blox 模块：UBX-only、归一化到 115200、10Hz、开启 NAV-PVT 输出。
 * 调用前 UART 必须已切到模块当前实际波特率（gps_autobaud 探测结果）。 */
static void ubx_config_ublox(void) {
    // 波特率切换 + UBX-only 已由 gps_autobaud 的 PUBX 命令完成。

    if (s_ubx_platform >= 9) {
        /* M9/M10：用 VALSET（10Hz + 使能 NAV-PVT + 禁 NMEA），对齐 BF */
        ubx_config_valset();
    } else {
        /* M8 及以下：传统命令（CFG-RATE + CFG-MSG 使能 NAV-PVT）。
         * CFG-MSG 用 3 字节形式（msgClass+msgID+rate），rate 作用于命令到达的当前端口（UART1），
         * 与 BF 的 ubloxSetMessageRate 完全一致。不要用 8 字节全目标形式（会把 UART1 置 0 禁用）。 */
        const uint8_t rate[6] = {0x64, 0x00, 0x01, 0x00, 0x01, 0x00};
        ubx_send(UBX_CLASS_CFG, UBX_CFG_RATE, rate, 6);
        const uint8_t msg3[3] = {0x01, 0x07, 0x01};  // NAV-PVT @1Hz 起，后续按 CFG-RATE 10Hz 输出
        ubx_send(UBX_CLASS_CFG, UBX_CFG_MSG, msg3, 3);
    }

    // CFG-GNSS：开启多星座（传统命令，M8/M9/M10 都支持）
    ubx_config_gnss();
}

/* UBX 帧流式解析状态机 */
typedef enum {
    UBX_S_SYNC1,
    UBX_S_SYNC2,
    UBX_S_CLASS,
    UBX_S_ID,
    UBX_S_LEN1,
    UBX_S_LEN2,
    UBX_S_PAYLOAD,
    UBX_S_CK_A,
    UBX_S_CK_B,
} ubx_state_t;

static ubx_state_t s_ubx_state = UBX_S_SYNC1;
static uint8_t  s_ubx_class = 0;
static uint8_t  s_ubx_id = 0;
static uint16_t s_ubx_len = 0;
static uint16_t s_ubx_idx = 0;
static uint8_t  s_ubx_ck_a = 0;
static uint8_t  s_ubx_ck_b = 0;
static uint8_t  s_ubx_payload[128];   // NAV-PVT 92 字节，留余量

/* 复位 UBX 帧解析状态机（波特率探测切换候选时调用，防止跨候选污染） */
static void ubx_state_reset(void) {
    s_ubx_state = UBX_S_SYNC1;
    s_ubx_class = 0;
    s_ubx_id = 0;
    s_ubx_len = 0;
    s_ubx_idx = 0;
    s_ubx_ck_a = 0;
    s_ubx_ck_b = 0;
}

/* 解析 MON-VER 的 hwVersion（payload 偏移 30，10 字节 ASCII），判断 M8/M9/M10 */
static void ubx_parse_mon_ver(const uint8_t *p, uint16_t len) {
    if (len < 40) return;  // swVersion(30) + hwVersion(10)
    char hw[11];
    memcpy(hw, &p[30], 10);
    hw[10] = '\0';
    uint32_t hwv = (uint32_t)strtoul(hw, NULL, 16);
    if (hwv == 0x00080000) {
        s_ubx_platform = 8;
    } else if (hwv == 0x00190000) {
        s_ubx_platform = 9;
    } else if (hwv == 0x000A0000) {
        s_ubx_platform = 10;
    } else if (hwv == 0x00070000) {
        s_ubx_platform = 7;
    }
    ESP_LOGI(TAG, "GPS platform: M%d (hw=0x%08X)", (unsigned)s_ubx_platform, (unsigned)hwv);
}

/* 解析 UBX-NAV-PVT（0x01 0x07，92 字节）到 GPS_Data */
static void ubx_parse_pvt(const uint8_t *p, uint16_t len) {
    if (len < 92) return;

    if (!s_gps_connected) {
        ESP_LOGI(TAG, "GPS: first NAV-PVT received, link up (fixType=%u numSV=%u)",
                 (unsigned)p[20], (unsigned)p[23]);
    }
    s_gps_connected = true;   // 收到 NAV-PVT，链路已建立

    int32_t lat = ubx_le32(&p[28]);   // 1e-7 度
    int32_t lon = ubx_le32(&p[24]);   // 1e-7 度
    int32_t height_mm = ubx_le32(&p[32]);
    int32_t vel_n = ubx_le32(&p[48]); // mm/s
    int32_t vel_e = ubx_le32(&p[52]); // mm/s
    int32_t vel_d = ubx_le32(&p[56]); // mm/s
    int32_t g_speed = ubx_le32(&p[60]);   // mm/s
    int32_t heading = ubx_le32(&p[64]);   // 1e-5 度

    uint8_t fix_type = p[20];
    uint8_t num_sv   = p[23];

    GPS_Data.Year  = (uint8_t)(ubx_le16(&p[4]) - 2000);
    GPS_Data.Month = p[6];
    GPS_Data.Day   = p[7];
    GPS_Data.Hour  = p[8];
    GPS_Data.Minute = p[9];
    GPS_Data.Second = (double)p[10];

    GPS_Data.Latitude = lat * 1e-7;
    GPS_Data.Longitude = lon * 1e-7;
    GPS_Data.Lat_Indicator = (lat >= 0) ? 'N' : 'S';
    GPS_Data.Lon_Indicator = (lon >= 0) ? 'E' : 'W';
    GPS_Data.Altitude = height_mm / 1000.0;

    GPS_Data.Velocity_North = vel_n / 1000.0;
    GPS_Data.Velocity_East = vel_e / 1000.0;
    GPS_Data.Velocity_Descend = vel_d / 1000.0;  // velD 向下为正，与既有约定一致

    double speed_m_s = g_speed / 1000.0;
    GPS_Data.Speed_knots = speed_m_s / 0.514444;  // m/s -> knots
    GPS_Data.Course = heading * 1e-5;

    GPS_Data.Num_Satellites = num_sv;

    // 定位有效判定：3D 及以上视为有效（等价于 NMEA 路径 RMC+GGA 同时有效）
    bool has_fix = (fix_type >= 3);
    GPS_Data.Status = has_fix ? 1 : 0;
    GPS_Data.RMC_Valid = has_fix ? 1 : 0;
    GPS_Data.GGA_Valid = has_fix ? 1 : 0;
    GPS_Data.RMC_Latitude = GPS_Data.Latitude;
    GPS_Data.RMC_Longitude = GPS_Data.Longitude;
    GPS_Data.GGA_Latitude = GPS_Data.Latitude;
    GPS_Data.GGA_Longitude = GPS_Data.Longitude;

    if (has_fix) {
        gps_invalid_count = 0;
    } else if (gps_invalid_count < UINT8_MAX) {
        gps_invalid_count++;
    }
}

/* 向状态机喂一个字节；返回 true 表示刚解析完一帧有效的 NAV-PVT */
static bool ubx_feed(uint8_t b) {
    switch (s_ubx_state) {
    case UBX_S_SYNC1:
        if (b == UBX_SYNC1) s_ubx_state = UBX_S_SYNC2;
        break;
    case UBX_S_SYNC2:
        if (b == UBX_SYNC2) {
            s_ubx_state = UBX_S_CLASS;
            s_ubx_ck_a = 0;
            s_ubx_ck_b = 0;
        } else if (b != UBX_SYNC1) {
            s_ubx_state = UBX_S_SYNC1;
        }
        break;
    case UBX_S_CLASS:
        s_ubx_class = b;
        s_ubx_ck_a = (uint8_t)(s_ubx_ck_a + b);
        s_ubx_ck_b = (uint8_t)(s_ubx_ck_b + s_ubx_ck_a);
        s_ubx_state = UBX_S_ID;
        break;
    case UBX_S_ID:
        s_ubx_id = b;
        s_ubx_ck_a = (uint8_t)(s_ubx_ck_a + b);
        s_ubx_ck_b = (uint8_t)(s_ubx_ck_b + s_ubx_ck_a);
        s_ubx_state = UBX_S_LEN1;
        break;
    case UBX_S_LEN1:
        s_ubx_len = b;
        s_ubx_ck_a = (uint8_t)(s_ubx_ck_a + b);
        s_ubx_ck_b = (uint8_t)(s_ubx_ck_b + s_ubx_ck_a);
        s_ubx_state = UBX_S_LEN2;
        break;
    case UBX_S_LEN2:
        s_ubx_len |= (uint16_t)((uint16_t)b << 8);
        s_ubx_ck_a = (uint8_t)(s_ubx_ck_a + b);
        s_ubx_ck_b = (uint8_t)(s_ubx_ck_b + s_ubx_ck_a);
        s_ubx_idx = 0;
        s_ubx_state = (s_ubx_len == 0) ? UBX_S_CK_A : UBX_S_PAYLOAD;
        break;
    case UBX_S_PAYLOAD:
        s_ubx_ck_a = (uint8_t)(s_ubx_ck_a + b);
        s_ubx_ck_b = (uint8_t)(s_ubx_ck_b + s_ubx_ck_a);
        if (s_ubx_idx < sizeof(s_ubx_payload)) {
            s_ubx_payload[s_ubx_idx] = b;
        }
        s_ubx_idx++;
        if (s_ubx_idx >= s_ubx_len) s_ubx_state = UBX_S_CK_A;
        break;
    case UBX_S_CK_A:
        s_ubx_state = (b == s_ubx_ck_a) ? UBX_S_CK_B : UBX_S_SYNC1;
        break;
    case UBX_S_CK_B:
        s_ubx_state = UBX_S_SYNC1;
        if (b == s_ubx_ck_b) {
            if (s_ubx_class == UBX_CLASS_NAV && s_ubx_id == UBX_ID_PVT) {
                ubx_parse_pvt(s_ubx_payload, s_ubx_len);
                return true;
            }
            if (s_ubx_class == UBX_CLASS_MON && s_ubx_id == UBX_ID_MON_VER) {
                s_ubx_mon_ver_got = true;   // 波特率探测成功
                ubx_parse_mon_ver(s_ubx_payload, s_ubx_len);
            }
            if (s_ubx_class == UBX_CLASS_ACK && s_ubx_id == UBX_ID_ACK_ACK) {
                s_ubx_ack_ack = true;        // 配置命令被模块接受
            }
        }
        break;
    }
    return false;
}



/**
 * @brief 初始化 GPS UART
 *        Initialize GPS UART
 * 
 * 配置并初始化 GPS UART，用于接收 GPS 数据。
 * Configure and initialize GPS UART for receiving GPS data.
 */
static void initUartGps(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_GPS_PORT, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_GPS_PORT, &uart_config);
    uart_set_pin(UART_GPS_PORT, UART_GPS_TXD_PIN, UART_GPS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
 * @brief GPS 数据接收任务
 *        GPS data receiving task
 * 
 * 从 GPS UART 端口读取数据，解析并处理 UBX 数据。
 * Read data from GPS UART port, parse and process UBX data.
 * 
 * @param arg 任务参数
 *            Task parameters
 */
static void rx_task_GPS(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);

    while (1) {
        const int rxBytes = uart_read_bytes(UART_GPS_PORT, data, RX_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            /* 纯 UBX 路径（对齐 BF：只支持 u-blox） */
            for (int i = 0; i < rxBytes; i++) {
                ubx_feed(data[i]);
            }

            if (s_data_ready_cb != NULL && is_current_gps_data_valid()) {
                s_data_ready_cb();
            }
        }
        // 如果没有数据读取，休眠一小段时间，避免任务占用 CPU
        // If no data is read, sleep for a short time to avoid CPU occupation
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
}

/**
 * @brief 初始化并启动 GPS 数据接收任务
 *        Initialize and start GPS data receiving task
 * 
 * 初始化 GPS UART 和相关任务，以定期接收 GPS 数据。
 * Initialize GPS UART and related tasks to periodically receive GPS data.
 */
void initSendGpsDataToCameraTask(void) {
    init_gps_data();
    initUartGps();

    // 探测模块实际波特率，并把模块切换为 UBX 协议、开启 NAV-PVT 输出（10Hz）。
    // 纯 UBX：rx_task_GPS 只解析 UBX 帧。
    gps_autobaud();
    ubx_config_ublox();

    xTaskCreate(rx_task_GPS, "uart_rx_task_GPS", 1024 * 4, NULL, 0, NULL);
    ESP_LOGI(TAG, "uart_rx_task_GPS are running\n");
}
