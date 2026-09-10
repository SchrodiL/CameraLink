/* SPDX-License-Identifier: MIT */

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "osd.h"
#include "camera_state.h"
#include "camera_enums.h"
#include "hardware_config.h"
#include "osd_config.h"
#include "channel_map.h"
#include "msp_uart.h"
#include "msp_protocol.h"
#include "msp_messages.h"

#define TAG "LOGIC_OSD"

/* MSPv2 原生帧；MSP2_SET_TEXT 是 MSPv2-only 命令 */
#define MSP_LINK_VERSION  MSP_V2_NATIVE

/* OSD 更新周期（毫秒） */
#define OSD_UPDATE_MS     1000

static msp_host_t s_msp;

/* 低电量报警图标闪烁相位：每个 OSD 刷新周期翻转一次（周期 1s → 0.5Hz 交替，即 1s 亮 1s 灭） */
static bool s_batt_blink_on = false;

/* ------------------------------------------------------------------ */
/* OSD 内容池：按槽位配置把相机状态组合成一条 OSD 文本（<= 16 字符）      */
/* ------------------------------------------------------------------ */

/* 分辨率枚举 -> OSD 紧凑缩写（统一用 1K/2K/4K/8K 命名） */
static const char *osd_res_abbr(uint8_t res)
{
    switch (res) {
    case 10:  return "1K";
    case 16:  return "4K";
    case 45:  return "2K";
    case 66:  return "1K";
    case 67:  return "2K";
    case 95:  return "2K";
    case 103: return "4K";
    case 109: return "4K";
    default:  return "--";
    }
}

/* fps_idx 枚举 -> 数字字符串 */
static const char *osd_fps_str(uint8_t fps_idx)
{
    switch (fps_idx) {
    case 1:  return "24";
    case 2:  return "25";
    case 3:  return "30";
    case 4:  return "48";
    case 5:  return "50";
    case 6:  return "60";
    case 7:  return "120";
    case 8:  return "240";
    case 10: return "100";
    case 19: return "200";
    default: return "--";
    }
}

/* 分辨率枚举 -> 拍摄比例（画幅）。视频 16:9 / 4:3 / 9:16；未知返回 NULL。 */
static const char *osd_aspect_abbr(uint8_t res)
{
    switch (res) {
    case 10:                       /* 1080P 16:9 */
    case 16:                       /* 4K 16:9 */
    case 45:                       /* 2.7K 16:9 */
        return "16:9";
    case 95:                       /* 2.7K 4:3 */
    case 103:                      /* 4K 4:3 */
        return "4:3";
    case 66:                       /* 1080P 9:16 */
    case 67:                       /* 2.7K 9:16 */
    case 109:                      /* 4K 9:16 */
        return "9:16";
    case 125:                      /* 自由裁切(Free Crop) 1:1，枚举值未公开，实测 */
        return "1:1";
    default:
        return NULL;
    }
}

/* 拍照画幅 -> OSD 显示名（4-L，3-M，2-S） */
static const char *osd_photo_format(uint8_t res)
{
    switch (res) {
    case 4: return "L";
    case 3: return "M";
    case 2: return "S";
    default: return NULL;
    }
}

/* 拍照比例 -> "4:3" / "16:9"（photo_ratio：0-4:3，1-16:9） */
static const char *osd_photo_ratio(uint8_t ratio)
{
    return ratio == 1 ? "16:9" : "4:3";
}

/* 相机模式枚举 -> OSD 紧凑缩写（纯大写：OSD 字体无小写，中文无法显示） */
static const char *osd_mode_abbr(uint8_t mode)
{
    switch (mode) {
    case CAMERA_MODE_NORMAL:          return "VIDEO";
    case CAMERA_MODE_PHOTO:           return "PHOTO";
    case CAMERA_MODE_TIMELAPSE:       return "TIMELAPSE";
    case CAMERA_MODE_HYPERLAPSE:
    case CAMERA_MODE_HYPERLAPSE_360:  return "HYPERLAP";
    case CAMERA_MODE_SLOW_MOTION:     return "SLOWMO";
    case CAMERA_MODE_LIVE_STREAMING:  return "LIVE";
    case CAMERA_MODE_UVC_STREAMING:   return "UVC";
    case CAMERA_MODE_SUPERNIGHT:      return "NIGHT";
    case CAMERA_MODE_SUBJECT_TRACKING: return "TRACK";
    case CAMERA_MODE_PANORAMIC_VIDEO_360:      return "PANO V";
    case CAMERA_MODE_SELFIE_360:               return "SELFIE";
    case CAMERA_MODE_PANORAMIC_PHOTO_360:      return "PANO P";
    case CAMERA_MODE_BOOST_VIDEO_360:          return "BOOST";
    case CAMERA_MODE_VORTEX_360:               return "VORTEX";
    case CAMERA_MODE_PANORAMIC_SUPERNIGHT_360: return "PANO N";
    case CAMERA_MODE_SINGLE_LENS_SUPERNIGHT_360: return "SL NIGHT";
    default:                         return "--";
    }
}

/* 1D06 mode_param 里相机下发的分辨率前缀 -> OSD 显示名：
 * 用户要 1K/2K 而不是 1080P/2.7K；4K/8K 原样透传。 */
typedef struct {
    const char *raw;   /* 相机下发的前缀，如 "1080P" */
    const char *osd;   /* OSD 上显示的名称，如 "1K" */
} osd_spec_res_map_t;

static const osd_spec_res_map_t s_spec_res_map[] = {
    { "1080P", "1K" },
    { "1080",  "1K" },
    { "2.7K",  "2K" },
};

/* 把 mode_param（如 "1080P30 Off"）改写成 OSD 可显示文本：
 * 1) 先整串转大写——OSD 字体只有大写字母，小写 f 会被渲染成箭头图标；
 * 2) 分辨率前缀统一改写为 1K/2K/4K/8K；
 * 3) 防抖关闭(Off)不显示，去掉末尾的 " OFF"。 */
static void osd_rewrite_spec(const char *param, char *out, size_t out_sz)
{
    char up[32];
    size_t i;
    for (i = 0; i < sizeof(up) - 1 && param[i] != '\0'; i++) {
        up[i] = (char)toupper((unsigned char)param[i]);
    }
    up[i] = '\0';

    bool matched = false;
    for (i = 0; i < sizeof(s_spec_res_map) / sizeof(s_spec_res_map[0]); i++) {
        size_t n = strlen(s_spec_res_map[i].raw);
        if (strncmp(up, s_spec_res_map[i].raw, n) == 0) {
            snprintf(out, out_sz, "%s%s", s_spec_res_map[i].osd, up + n);
            matched = true;
            break;
        }
    }
    if (!matched) {
        snprintf(out, out_sz, "%s", up);
    }

    /* 防抖关闭(Off)不显示：去掉末尾的 " OFF" */
    char *off = strstr(out, " OFF");
    if (off != NULL) {
        *off = '\0';
    }
}

/* 判断 device_id 是否为某型号码（相机发送的字节序不定，高低 16 位都试） */
static bool osd_dev_id_is(uint32_t dev, uint16_t code)
{
    return (uint16_t)dev == code || (uint16_t)(dev >> 16) == code;
}

/* 把相机型号缩写成 OSD 紧凑名称：
 * Action 6 -> OA6，Action 5(Pro) -> OA5，Action 4 -> OA4，
 * Osmo Nano -> ON，Osmo 360 -> O360。
 * 优先用连接请求帧里的 device_id（可靠），失败再按 product_id
 * 名称子串猜测（大小写不敏感，容忍填充差异）；未知型号返回 NULL。 */
static const char *osd_cam_abbr(const char *name, uint32_t device_id)
{
    if (device_id != 0) {
        if (osd_dev_id_is(device_id, 0xFF55)) return "OA6";
        if (osd_dev_id_is(device_id, 0xFF44)) return "OA5";
        if (osd_dev_id_is(device_id, 0xFF33)) return "OA4";
        if (osd_dev_id_is(device_id, 0xFF66)) return "O360";
    }

    char up[17];
    int i;
    for (i = 0; i < 16 && name[i] != '\0'; i++) {
        up[i] = (char)toupper((unsigned char)name[i]);
    }
    up[i] = '\0';

    if (strstr(up, "ACTION 6") || strstr(up, "ACTION6")) return "OA6";
    if (strstr(up, "ACTION 5") || strstr(up, "ACTION5")) return "OA5";
    if (strstr(up, "ACTION 4") || strstr(up, "ACTION4")) return "OA4";
    if (strstr(up, "NANO"))     return "ON";
    if (strstr(up, "360"))      return "O360";
    return NULL;
}

static void compose_item(osd_item_t item, const camera_state_t *st, char *out)
{
    switch (item) {
    case OSD_ITEM_REC: {
        if (st->recording) {
            uint32_t m = st->rec_seconds / 60;
            uint32_t s = st->rec_seconds % 60;
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%c%u:%02u",
                     (char)MSP_OSD_SYM_REC, (unsigned)m, (unsigned)s);
        } else if (st->connected) {
            /* 待机时显示拍摄模式（insta360 无模式概念，回退 STBY）。
             * 优先用 1D06 下发的模式名（如 PORTRAIT），旧协议无此字段时回退枚举映射。 */
            if (st->protocol == CAM_PROTO_DJI) {
                if (st->mode_name[0] != '\0') {
                    char up[21];
                    size_t i;
                    for (i = 0; i < sizeof(up) - 1 && st->mode_name[i] != '\0'; i++) {
                        up[i] = (char)toupper((unsigned char)st->mode_name[i]);
                    }
                    up[i] = '\0';
                    msp_osd_sanitize(out, up, OSD_CUSTOM_MSG_MAX_LEN);
                } else {
                    snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%s", osd_mode_abbr(st->mode));
                }
            } else {
                snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "STBY");
            }
        } else {
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "NO CAM");
        }
        break;
    }

    case OSD_ITEM_GPS: {
        if (st->gps_connected) {
            /* 卫星图标(0x1E) + 卫星数；连接但未定位时显示 0 颗星 */
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%c%u",
                     (char)MSP_OSD_SYM_SAT, (unsigned)st->satellites);
        } else {
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "NO GPS");
        }
        break;
    }

    case OSD_ITEM_SPEED_ALT: {
        uint32_t spd10 = (uint32_t)(st->speed_ms * 10.0 + 0.5);   /* 12.3 m/s -> 123 */
        if (spd10 > 999) spd10 = 999;                              /* 上限 99.9 m/s */
        unsigned spd_int = (unsigned)(spd10 / 10);
        unsigned spd_frac = (unsigned)(spd10 % 10);

        int alt = (int)(st->altitude_m + 0.5);
        if (alt < 0) alt = 0;
        if (alt > 9999) alt = 9999;

        char tmp[32];
        /* 速度图标(0x70) + 速度，海拔图标(0x7F) + 海拔 */
        snprintf(tmp, sizeof(tmp), "%c%u.%u %c%um",
                 (char)MSP_OSD_SYM_SPEED, spd_int, spd_frac,
                 (char)MSP_OSD_SYM_ALTITUDE, (unsigned)alt);
        msp_osd_sanitize(out, tmp, OSD_CUSTOM_MSG_MAX_LEN);
        break;
    }

    case OSD_ITEM_BATTERY: {
        if (st->connected && st->battery_pct > 0) {
            uint8_t alarm = osd_config_get_batt_alarm();
            unsigned pct = (unsigned)st->battery_pct;

            if (alarm > 0 && pct < alarm) {
                /* 低电量报警：图标切换为 0x97 电池符号，并随刷新周期闪烁（数字常显） */
                char icon = s_batt_blink_on ? (char)MSP_OSD_SYM_MAIN_BATT : ' ';
                snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%c%u%%", icon, pct);
            } else {
                /* 正常：0x90..0x96 电量格按百分比均分为 7 档（满→空） */
                unsigned level = (pct * 7U) / 101U;
                char icon = (char)(MSP_OSD_SYM_BATT_EMPTY - level);
                snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%c%u%%", icon, pct);
            }
        } else {
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%c--",
                     (char)MSP_OSD_SYM_MAIN_BATT);
        }
        break;
    }

    case OSD_ITEM_SPEC: {
        if (!st->connected) {
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "NO SPEC");
        } else if (st->res == 2 || st->res == 3 || st->res == 4) {
            if (st->real_time_countdown > 0) {
                /* 实时倒计时激活：整个元素只显示倒计时秒数 */
                snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%uS",
                         (unsigned)st->real_time_countdown);
            } else {
                /* 常态：画幅 + 比例 + 连拍 + 定时档位。
                 * fps_idx=1 单拍；>1 为连拍张数，record_time 为连拍时限(毫秒)。 */
                const char *fmt = osd_photo_format(st->res);
                const char *ratio = osd_photo_ratio(st->photo_ratio);
                char tmp[40];
                if (st->fps_idx > 1 && st->record_time > 0) {
                    unsigned sec = (unsigned)(st->record_time / 1000);
                    snprintf(tmp, sizeof(tmp), "%s %s %uP%uS",
                             fmt, ratio, (unsigned)st->fps_idx, sec);
                } else {
                    /* 单拍：不显示 OFF */
                    snprintf(tmp, sizeof(tmp), "%s %s", fmt, ratio);
                }
                /* 追加定时档位（0.5/1/2/3/5/10 秒），未开定时(0)不显示 */
                if (st->photo_countdown_ms > 0) {
                    unsigned ms = st->photo_countdown_ms;
                    unsigned sec = ms / 1000;
                    unsigned tenth = (ms % 1000) / 100;
                    size_t n = strlen(tmp);
                    if (tenth == 0) {
                        snprintf(tmp + n, sizeof(tmp) - n, " %uS", sec);
                    } else {
                        snprintf(tmp + n, sizeof(tmp) - n, " %u.%uS", sec, tenth);
                    }
                }
                msp_osd_sanitize(out, tmp, OSD_CUSTOM_MSG_MAX_LEN);
            }
        } else {
            char base[32];
            if (st->mode_param[0] != '\0') {
                /* 新协议(1D06)提供模式参数字符串，如 "8K30"，含分辨率与帧率；
                 * 分辨率前缀统一改写成 1K/2K/4K/8K。 */
                osd_rewrite_spec(st->mode_param, base, sizeof(base));
            } else {
                snprintf(base, sizeof(base), "%s/%s",
                         osd_res_abbr(st->res), osd_fps_str(st->fps_idx));
            }

            /* 拍摄比例：优先从 1D02 的 video_resolution 枚举推导；
             * 8K 的枚举值未公开，但 8K 只有 16:9，按 mode_param 开头判断固定显示。 */
            const char *aspect = osd_aspect_abbr(st->res);
            if (aspect == NULL &&
                st->mode_param[0] == '8' &&
                (st->mode_param[1] == 'K' || st->mode_param[1] == 'k')) {
                aspect = "16:9";
            }

            char tmp[40];
            if (aspect != NULL) {
                /* 比例放到第二个：res+fps 之后、EIS 之前 */
                char *sp = strchr(base, ' ');
                if (sp != NULL) {
                    *sp = '\0';
                    snprintf(tmp, sizeof(tmp), "%s %s %s", base, aspect, sp + 1);
                } else {
                    snprintf(tmp, sizeof(tmp), "%s %s", base, aspect);
                }
            } else {
                snprintf(tmp, sizeof(tmp), "%s", base);
            }
            msp_osd_sanitize(out, tmp, OSD_CUSTOM_MSG_MAX_LEN);
        }
        break;
    }

    case OSD_ITEM_NAME_STORAGE: {
        if (!st->connected || st->name[0] == '\0') {
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "NO CAM");
            break;
        }
        char tmp[32];
        if (st->protocol == CAM_PROTO_INSTA360) {
            /* insta360：型号缩写 + 剩余（拍照=张数 P，录像=分钟 m） */
            if (st->remain_photos > 0) {
                snprintf(tmp, sizeof(tmp), "%s %uP", st->name, (unsigned)st->remain_photos);
            } else {
                unsigned min = (unsigned)(st->remain_time_s / 60);
                snprintf(tmp, sizeof(tmp), "%s %um", st->name, min);
            }
        } else {
            unsigned gb = (unsigned)(st->remain_capacity_mb / 1024);
            const char *abbr = osd_cam_abbr(st->name, st->device_id);
            if (abbr != NULL) {
                snprintf(tmp, sizeof(tmp), "%s %uG", abbr, gb);
            } else {
                snprintf(tmp, sizeof(tmp), "%.10s %uG", st->name, gb);
            }
        }
        msp_osd_sanitize(out, tmp, OSD_CUSTOM_MSG_MAX_LEN);
        break;
    }

    default: {
        snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "---");
        break;
    }
    }
}

static void compose_osd(const camera_state_t *st,
                        char msgs[][OSD_CUSTOM_MSG_MAX_LEN + 1])
{
    osd_item_t slots[OSD_SLOT_COUNT];
    osd_config_get_slots(slots);

    for (int i = 0; i < OSD_SLOT_COUNT; i++) {
        compose_item(slots[i], st, msgs[i]);
    }
}

/* ------------------------------------------------------------------ */
/* 把 4 条文本用 MSP2_SET_TEXT 发给飞控                                    */
/* ------------------------------------------------------------------ */
static void send_osd_msgs(char msgs[][OSD_CUSTOM_MSG_MAX_LEN + 1])
{
    for (int i = 0; i < OSD_CUSTOM_MSG_COUNT; i++) {
        /* OSD 字体只能显示可打印 ASCII，先过滤一遍 */
        char safe[OSD_CUSTOM_MSG_MAX_LEN + 1];
        msp_osd_sanitize(safe, msgs[i], OSD_CUSTOM_MSG_MAX_LEN);

        uint8_t payload[OSD_CUSTOM_MSG_MAX_LEN + 2];
        uint16_t len = msp_build_custom_msg(payload, (uint8_t)i, safe);

        /* 只发不等应答；飞控的应答由 flush_rx 丢弃 */
        int rc = msp_host_send(&s_msp, MSP2_SET_TEXT, payload, len);
        if (rc != MSP_HOST_OK) {
            ESP_LOGW(TAG, "SET_TEXT custom_osd[%d] failed (%d)", i, rc);
        }
    }
    msp_uart_flush_rx();
}

/* ------------------------------------------------------------------ */

static void osd_task(void *arg)
{
    (void)arg;
    TickType_t last_osd = 0;
    for (;;) {
        /* 读 RC 通道（~10Hz），驱动通道映射。与 OSD 写共用同一 UART，顺序执行。 */
        msp_packet_t reply;
        msp_decoder_init(&s_msp.dec);   /* 安全复位解析状态机 */
        if (msp_host_request(&s_msp, MSP_RC, NULL, 0, &reply, 50) == MSP_HOST_OK) {
            uint16_t ch[16];
            if (msp_decode_rc(&reply, ch)) {
                channel_map_feed(ch);
            }
        }

        /* OSD 文本写保持 ~1Hz（OSD_UPDATE_MS）。 */
        TickType_t now = xTaskGetTickCount();
        if (now - last_osd >= pdMS_TO_TICKS(OSD_UPDATE_MS)) {
            last_osd = now;
            s_batt_blink_on = !s_batt_blink_on;
            camera_state_refresh();
            const camera_state_t *st = camera_state_get();

            char msgs[OSD_CUSTOM_MSG_COUNT][OSD_CUSTOM_MSG_MAX_LEN + 1];
            compose_osd(st, msgs);
            send_osd_msgs(msgs);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int osd_logic_init(void)
{
    msp_uart_config_t cfg = {
        .uart_num  = MSP_UART_NUM,
        .tx_pin    = MSP_TX_PIN,
        .rx_pin    = MSP_RX_PIN,
        .baud_rate = MSP_BAUD_RATE,
    };

    esp_err_t err = msp_uart_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "msp_uart_init failed: %s", esp_err_to_name(err));
        return -1;
    }
    msp_uart_bind(&s_msp, MSP_LINK_VERSION);

    BaseType_t r = xTaskCreate(osd_task, "osd_task", 4096, NULL, 1, NULL);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "failed to create osd_task");
        return -1;
    }

    ESP_LOGI(TAG, "OSD task started (UART%u, %d baud, v%d)",
             (unsigned)MSP_UART_NUM, MSP_BAUD_RATE, MSP_LINK_VERSION);
    return 0;
}
