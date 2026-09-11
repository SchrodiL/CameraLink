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
#include "fc_msp.h"          /* OSD 文本交给 MSP 链路去发 */
#include "msp_protocol.h"    /* OSD_CUSTOM_MSG_* 常量 */
#include "msp_messages.h"    /* msp_osd_sanitize */

#define TAG "LOGIC_OSD"

/* OSD 文本写周期（毫秒）。取 500ms（2Hz）与相机的 2Hz 状态推送对齐——
 * 原先 1Hz 会让画面比相机状态慢一拍（最坏滞后 1 秒多）。 */
#define OSD_UPDATE_MS     500

/* 任务轮询粒度（毫秒）。越小，OSD 到点的抖动越小。 */
#define OSD_POLL_MS       50

/* 低电量图标闪烁的半周期（毫秒）：1 秒亮、1 秒灭，与 OSD 写周期解耦，
 * 这样提高写频率不会让图标闪得更快。 */
#define OSD_BLINK_MS      1000

/* 充电动画步进间隔（毫秒）：2Hz，每秒推进两格，7 格一轮约 3.5s。 */
#define OSD_CHRG_MS       500

/* 电量文字档位 → OSD 字符串；BATT_LABEL_NONE 返回 NULL（改用百分比显示）。 */
static const char *batt_label_str(battery_label_t l)
{
    switch (l) {
    case BATT_LABEL_FULL:   return "FULL";
    case BATT_LABEL_HIGH:   return "HIGH";
    case BATT_LABEL_MEDIUM: return "MEDIUM";
    case BATT_LABEL_LOW:    return "LOW";
    default:                return NULL;
    }
}

/* 电量档位 → OSD 电量格图标（0x90 满 / 0x92 / 0x94 / 0x97 低电量报警符号）。
 * 相机只按档位上报电量，图标就用固定格数，而不是把区间下界当成精确百分比去算格数
 * ——那样同一档位内的边界抖动（挡位边界 ±1%）会让图标来回跳。
 * BATT_LABEL_LOW 直接落到 0x97（报警符号），与 DJI 侧报警阈值触发时同一个图标。
 * BATT_LABEL_NONE（DJI 有精确百分比）返回 0，由调用方按百分比分档。 */
static uint8_t batt_icon_for_label(battery_label_t l)
{
    switch (l) {
    case BATT_LABEL_FULL:   return MSP_OSD_SYM_BATT_FULL;        /* 0x90 */
    case BATT_LABEL_HIGH:   return MSP_OSD_SYM_BATT_FULL + 2;    /* 0x92 */
    case BATT_LABEL_MEDIUM: return MSP_OSD_SYM_BATT_FULL + 4;    /* 0x94 */
    case BATT_LABEL_LOW:    return MSP_OSD_SYM_MAIN_BATT;        /* 0x97 */
    default:                return 0;
    }
}

/* 低电量报警 / 低电量档位 图标闪烁相位：1Hz 翻转（1s 亮 1s 灭），
 * 与 OSD 写频率解耦，提高写频率不会让图标闪得更快。 */
static bool s_batt_blink_on = false;

/* 充电动画相位 0..(CHRG_ANIM_STEPS-1)，每 OSD_CHRG_MS 推进一格。
 * 图标 = 0x96 - 相位，即从最空的一格逐格填到最满，到头回卷，形成填充动画。
 * 独立计时（2Hz），不跟随报警闪烁的 1Hz。 */
#define CHRG_ANIM_STEPS (MSP_OSD_SYM_BATT_EMPTY - MSP_OSD_SYM_BATT_FULL + 1)   /* 7 */
static uint8_t s_chrg_anim = 0;

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
            /* 待机时显示拍摄模式。
             * 两种协议都可能下发模式名（DJI 1D06 的 mode_name / insta360 的模式码），
             * 有名字就优先用，否则回退各自的枚举映射 / STBY。 */
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
            } else if (st->mode_name[0] != '\0') {
                /* insta360：模式表给出的名称，已是全大写 ASCII，直接显示。
                 * 用显式精度截断到 OSD 宽度（mode_name 缓冲区比 OSD 长）。 */
                snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%.*s",
                         OSD_CUSTOM_MSG_MAX_LEN, st->mode_name);
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
        snprintf(tmp, sizeof(tmp), "%c%u.%u %c%uM",
                 (char)MSP_OSD_SYM_SPEED, spd_int, spd_frac,
                 (char)MSP_OSD_SYM_ALTITUDE, (unsigned)alt);
        msp_osd_sanitize(out, tmp, OSD_CUSTOM_MSG_MAX_LEN);
        break;
    }

    case OSD_ITEM_GPS_LAT:
    case OSD_ITEM_GPS_LON: {
        const bool is_lat = (item == OSD_ITEM_GPS_LAT);
        const char sym = is_lat ? (char)MSP_OSD_SYM_LAT : (char)MSP_OSD_SYM_LON;

        if (!st->gps_connected) {
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "NO GPS");
            break;
        }
        if (!st->gps_valid) {
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%c--", sym);
            break;
        }

        /* 单独占一条消息，16 字符足够——固定 6 位小数：
         * 最坏 "-179.999999" 是 11 字符，加 1 个标志共 12，余量充足。 */
        char tmp[24];
        snprintf(tmp, sizeof(tmp), "%c%.6f", sym, is_lat ? st->lat : st->lon);
        msp_osd_sanitize(out, tmp, OSD_CUSTOM_MSG_MAX_LEN);
        break;
    }

    case OSD_ITEM_BATTERY: {
        if (st->connected && st->charging) {
            /* 充电中：相机此时不上报电量档位（insta360 的心跳固定为充电标记值），
             * 显示充电图标 + CHRG，而不是给出一个错误的百分比。
             * 图标从最空(0x96)逐格填到最满(0x90)再回卷，2Hz 填充动画。 */
            char icon = (char)(MSP_OSD_SYM_BATT_EMPTY - s_chrg_anim);
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%cCHRG", icon);
        } else if (st->connected && st->battery_pct > 0) {
            char ptxt[12];
            char icon;
            bool blink = false;

            const char *lbl = batt_label_str(st->battery_label);
            if (lbl != NULL) {
                /* 相机只按挡位上报（insta360）：图标由挡位定死。区间本身有 11~26% 的宽度，
                 * 写成 "25-49%" 容易被误读成精确值；档位词更诚实也更省 OSD 宽度。
                 * LOW 档即相机自己的低电量告警，直接用 0x97 报警符号并闪烁。 */
                snprintf(ptxt, sizeof(ptxt), "%s", lbl);
                icon  = (char)batt_icon_for_label(st->battery_label);
                blink = (st->battery_label == BATT_LABEL_LOW);
            } else {
                unsigned pct = (unsigned)st->battery_pct;
                if (st->battery_hi > st->battery_pct) {
                    snprintf(ptxt, sizeof(ptxt), "%u-%u%%", pct, (unsigned)st->battery_hi);
                } else {
                    snprintf(ptxt, sizeof(ptxt), "%u%%", pct);
                }

                /* DJI 有精确百分比：0x90..0x96 七格线性映射 0-100%
                 * （100%→0x90，25%→0x95，0%→0x96）。
                 * 用户配置的报警阈值在此之上覆盖：≤阈值一律换成 0x97 并闪烁报警。
                 * insta360 走上面的档位分支，**不读** WebUI 阈值——相机只按挡位上报，
                 * 可配阈值会被吸附到挡位边界（设 26 和设 50 效果相同），细调没有意义。 */
                unsigned level = (pct * 7U) / 101U;
                icon = (char)(MSP_OSD_SYM_BATT_EMPTY - level);

                uint8_t alarm = (st->protocol == CAM_PROTO_INSTA360)
                                ? 0 : osd_config_get_batt_alarm();
                if (alarm > 0 && pct <= alarm) {
                    icon  = (char)MSP_OSD_SYM_MAIN_BATT;   /* 0x97 报警符号 */
                    blink = true;
                }
            }

            /* 闪烁：灭相用空格顶替图标，文字常显 */
            if (blink && !s_batt_blink_on) {
                icon = ' ';
            }
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%c%s", icon, ptxt);
        } else {
            /* 未连接：最空的一格 + 占位符 */
            snprintf(out, OSD_CUSTOM_MSG_MAX_LEN + 1, "%c--",
                     (char)MSP_OSD_SYM_BATT_EMPTY);
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
                snprintf(tmp, sizeof(tmp), "%s %uM", st->name, min);
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
/* 把 4 条文本交给 MSP 链路的队列，由 fc_msp 的任务负责实际发送            */
/* ------------------------------------------------------------------ */
static void send_osd_msgs(char msgs[][OSD_CUSTOM_MSG_MAX_LEN + 1])
{
    for (int i = 0; i < OSD_CUSTOM_MSG_COUNT; i++) {
        /* OSD 字体只能显示可打印 ASCII 且大小写敏感，先过滤+转大写 */
        char safe[OSD_CUSTOM_MSG_MAX_LEN + 1];
        msp_osd_sanitize(safe, msgs[i], OSD_CUSTOM_MSG_MAX_LEN);

        /* 只入队不发送：MSP 链路由 fc_msp 的任务独占（宿主不是线程安全的），
         * 这里绝不直接碰 UART。非阻塞，队列满会丢弃并由下一轮重发。 */
        fc_msp_send_osd_text((uint8_t)i, safe);
    }
}

/* ------------------------------------------------------------------ */

static void osd_task(void *arg)
{
    (void)arg;
    TickType_t last_osd = 0;
    TickType_t last_blink = 0;
    TickType_t last_chrg = 0;
    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* 到点就合成一轮 OSD 文本并交给 fc_msp 发送。
         * 本任务不再做串口 I/O，所以这里不会因飞控不应答而顺延。 */
        if (now - last_osd >= pdMS_TO_TICKS(OSD_UPDATE_MS)) {
            last_osd = now;

            /* 报警闪烁 1Hz：与 OSD 写频率解耦，提高写频率不会让图标闪得更快。 */
            if (now - last_blink >= pdMS_TO_TICKS(OSD_BLINK_MS)) {
                last_blink = now;
                s_batt_blink_on = !s_batt_blink_on;
            }

            /* 充电动画 2Hz：独立计时，比报警闪烁快一倍。 */
            if (now - last_chrg >= pdMS_TO_TICKS(OSD_CHRG_MS)) {
                last_chrg = now;
                s_chrg_anim = (uint8_t)((s_chrg_anim + 1) % CHRG_ANIM_STEPS);
            }

            camera_state_refresh();
            const camera_state_t *st = camera_state_get();

            char msgs[OSD_CUSTOM_MSG_COUNT][OSD_CUSTOM_MSG_MAX_LEN + 1];
            compose_osd(st, msgs);
            send_osd_msgs(msgs);

            /* 发完再取一次时间，避免上面的串口写入时间被算进下一个周期。 */
            now = xTaskGetTickCount();
            last_osd = now;
        }

        /* RC 通道轮询已迁到 fc_msp 的任务（它独占 MSP UART）。
         * 本任务只负责合成文本，不再碰串口。 */
        vTaskDelay(pdMS_TO_TICKS(OSD_POLL_MS));
    }
}

int osd_logic_init(void)
{
    /* MSP UART 不在这里初始化 —— 它归 fc_msp 管（见 fc_msp_init）。
     * app_main 必须先调 fc_msp_init()，否则 send_osd_msgs 入队的文本没人发。 */
    BaseType_t r = xTaskCreate(osd_task, "osd_task", 4096, NULL, 1, NULL);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "failed to create osd_task");
        return -1;
    }

    ESP_LOGI(TAG, "OSD task started");
    return 0;
}
