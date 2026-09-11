/* SPDX-License-Identifier: MIT
 * Copyright (c) 2025 Cameron Coward (insta360-m5stick-remote)
 *
 * insta360 remote logic (BLE GATT server).
 * Ported from the insta360 M5StickC remote project:
 *   https://github.com/marcelpallares/insta360-m5stick-remote
 * Rewritten on top of the raw esp_ble_gatts_* API for ESP-IDF (no Arduino).
 */

#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "insta360.h"

#define TAG "INSTA360"

/* Insta360 GPS Remote service/characteristic UUIDs — 128-bit form, matching the
 * official remote exactly. The camera (esp. Ace Pro 2) matches the 128-bit UUID,
 * not the 16-bit shorthand, so broadcasting/registering 16-bit would leave it
 * stuck in pairing. Bytes are little-endian (as used on-air and in uuid128). */
#define INSTA360_UUID128(lo) \
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, lo, 0xCE, 0x00, 0x00

#define GATTS_NUM_HANDLE 8

/* NVS storage for the learned wake payload. */
#define INSTA360_NVS_NAMESPACE "insta360"
#define INSTA360_NVS_KEY_WAKE "wake_payload"
#define INSTA360_NVS_KEY_MODEL "model"

/* One-shot scan duration to discover the camera name (15 s, units of 0.625 ms). */
#define CAMERA_NAME_SCAN_DURATION 24000

/* 9-byte command payloads. cmd byte[7]=功能, byte[8]=子动作。
 * 空口抓包验证（nRF52840 sniffer）：
 *   byte[7]=0x02, byte[8]=0x00 → 相机回 0x0e/0x0f（开始/停止录制），即快门
 *   byte[7]=0x01, byte[8]=0x00 → 模式预切换（见下）
 *   byte[7]=0x00, byte[8]=0x00 → 睡眠/唤醒切换（同一按键 toggle，链路保持）
 *   byte[7]=0x00, byte[8]=0x03 → 关机（相机主动断开链路）
 *
 * 模式切换是**两步**（官方遥控实测：两条命令间隔约 1.1 秒）：
 *   byte[7]=0x01 → 相机进入「预切换」，只显示候选模式，**并不真正切换**
 *   byte[7]=0x02 → 确认，模式这时才生效（随后相机推新的 0x10 规格串）
 * 因此只发 0x01 会出现「遥控/OSD 显示变了、相机实际没变」的现象。
 * 注意 0x02 有双重语义：普通状态下是快门，预切换状态下是确认。 */
static const uint8_t SHUTTER_CMD[]    = {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x02, 0x00};
static const uint8_t MODE_NEXT_CMD[]  = {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x01, 0x00};
/* 睡眠/唤醒：同一个码，相机的状态决定是睡还是醒（insta3 对照抓包实证）。 */
static const uint8_t SLEEP_WAKE_CMD[] = {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x00, 0x00};
/* 关机：实测发出后约 0.8 秒链路被 LL_TERMINATE_IND 断开（insta2/insta4 各一次）。 */
static const uint8_t POWER_OFF_CMD[]  = {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x00, 0x03};

/* 10-byte Insta360 iBeacon UUID prefix ("ORBIT"). */
static const uint8_t ORBIT_UUID[10] = {0x09, 0x4F, 0x52, 0x42, 0x49, 0x54, 0x09, 0xFF, 0x0F, 0x00};

/* 大小写不敏感的前缀匹配（定义在下方，此处前置声明供机型查表使用）。 */
static bool prefix_ieq(const char *name, uint8_t len, const char *prefix);

/* 相机广播名可能带 "Insta360" 品牌前缀（如 "Insta360 Ace Pro 2 A1B2C3"），
 * 而型号表里存的是不带品牌的型号名（"Ace Pro 2"）。前缀匹配前先剥掉品牌，
 * 否则所有型号都匹配不上，OSD 设备名会退化成 NO CAM。
 * 返回剥掉品牌后的起始指针，并把 *len 减掉相应长度；没有品牌前缀时原样返回。 */
static const char *strip_brand_prefix(const char *model, size_t *len) {
    static const char brand[] = "Insta360";
    const size_t blen = sizeof(brand) - 1;

    if (*len > blen && prefix_ieq(model, (uint8_t)*len, brand)) {
        size_t i = blen;
        while (i < *len && model[i] == ' ') {
            i++;
        }
        *len -= i;
        return model + i;
    }
    return model;
}

/* GATT server state. */
static uint16_t s_gatts_if = ESP_GATT_IF_NONE;
static uint16_t s_service_handle = 0;
static uint16_t s_write_char_handle = 0;
static uint16_t s_notify_char_handle = 0;
static uint8_t s_add_step = 0;  // 0=idle, 1=write char added, 2=notify char added

/* Connection / recording state. */
static bool s_connected = false;
/* 唤醒信标正在发送中。唤醒会阻塞到相机连上（最长 15 秒），
 * 用来阻止重入把控制队列堵上一串长等待。只在控制任务里访问，无需加锁。 */
static bool s_waking = false;
static uint16_t s_conn_id = 0;
static bool s_recording = false;
static uint32_t s_last_timer_ms = 0;
static uint32_t s_rec_start_ms = 0;
static uint32_t s_rec_seconds = 0;   /* 相机下发的精确录制秒数（.HH:MM:SS 解析而来） */
static char s_mode_str[32] = "";     /* 相机下发的模式描述，如 "4K|30|DEW" */
static uint32_t s_remain_minutes = 0; /* 剩余录制时长（分钟），如 "22m" */
static uint32_t s_remain_photos = 0;  /* 拍照模式剩余张数，如 "837" */
static uint8_t s_battery_lo = 0;      /* 电量区间下界：未充电时由心跳 p3 查表得来 */
static uint8_t s_battery_hi = 0;      /* 电量区间上界（0 = 无区间信息） */
static battery_label_t s_battery_label = BATT_LABEL_NONE;  /* OSD 显示的档位词 */
static bool s_charging = false;       /* 充电中（心跳 p3 == INSTA360_CHARGING_CODE） */
static uint8_t s_status02[5] = {0};    /* 最近一次 type 0x02 状态包的原始 5 字节 */
/* 心跳包里代表「充电中」的固定 p3 值。实测：任意电量下插上充电器/放进充电仓，
 * p3 恒为该值（35% 时由 0x08 跳为 0x0B），此时相机不再上报电量档位。 */
#define INSTA360_CHARGING_CODE 0x0B

/* 电量挡位表：相机按挡位上报，每个 p3 值覆盖一段电量（越接近没电分得越细）。
 * 边界全部为实测跳变点，且已完整覆盖 0~100%：
 *     0x07 ← 0x06 在 10%    0x08 ← 0x07 在 25%
 *     0x09 ← 0x08 在 50%    0x0A ← 0x09 在 75%
 * 宽度 11 / 14 / 25 / 25 / 26。0x06 是最低挡，其下不存在更低挡位。
 *
 * 满电实测为 0x0A（不是 0x0B），说明 0x0B 不占用电量挡位 —— 它只作充电标记
 * （见 INSTA360_CHARGING_CODE）。因此本表可覆盖全部电量，无需再留余量。 */
typedef struct {
    uint8_t code;
    uint8_t lo;
    uint8_t hi;
    battery_label_t label;   /* OSD 显示的档位词（最低两挡都归 LOW） */
} insta360_batt_level_t;

static const insta360_batt_level_t INSTA360_BATT_LEVELS[] = {
    { 0x06,  0,  10, BATT_LABEL_LOW    },   /* 最低挡，其下无更低挡位（0~10% 同挡） */
    { 0x07, 11,  24, BATT_LABEL_LOW    },
    { 0x08, 25,  49, BATT_LABEL_MEDIUM },
    { 0x09, 50,  74, BATT_LABEL_HIGH   },
    { 0x0A, 75, 100, BATT_LABEL_FULL   },
};

static const insta360_batt_level_t *insta360_batt_level(uint8_t code)
{
    for (size_t i = 0; i < sizeof(INSTA360_BATT_LEVELS) / sizeof(INSTA360_BATT_LEVELS[0]); i++) {
        if (INSTA360_BATT_LEVELS[i].code == code) {
            return &INSTA360_BATT_LEVELS[i];
        }
    }
    return NULL;
}

static uint16_t s_mode_key = 0xFFFF;   /* 模式键 (payload[3]<<8)|payload[4]，0xFFFF=未知 */

/* ---------------------------------------------------------------------------
 * 拍摄模式 → OSD 名称
 *
 * 模式来自相机 type 0x02 状态包，由 **两个字节共同**标识：
 *   payload[3] = 模式组；payload[4] = 组内类型：**0x60 = 视频类，0x70 = 照片类**
 *   （实测规律：0x01/0x60=录像 且 0x01/0x70=拍照；0x03/0x60=循环录影 且
 *     0x03/0x70=间隔拍照；0x05/0x60=移动延时 且 0x05/0x70=星空延时）
 * payload[4]=0x50 的是固定内容心跳包（00 54 00 0a 50），不含模式。
 *
 * 因此**不能用 payload[3] 单字节当键** —— 同一组下 /0x60 与 /0x70 是不同模式。
 *
 * 该包**周期性**发送（间隔十几秒），不是一切模式就发；切换过快可能收不到。
 *
 * 判据：用户实测标注 + 空口抓包的「剩余量单位」（拍照类张数 / 录像类分钟 / 延时类小时）。
 * 名称用全大写（OSD 字体无小写，且与 DJI 侧命名风格一致）。
 * --------------------------------------------------------------------------- */
typedef struct {
    uint8_t code3;
    uint8_t code4;
    const char *name;
} insta360_mode_entry_t;

static const insta360_mode_entry_t INSTA360_MODES[] = {
    /* ---- 视频类（payload[4]=0x60）---- */
    { 0x01, 0x60, "VIDEO"     },  /* 录像        剩余量 21m                */
    { 0x03, 0x60, "LOOP"      },  /* 循环录影                              */
    { 0x04, 0x60, "TIMELAPSE" },  /* 延时摄影    剩余量 15h56m（小时级）    */
    { 0x05, 0x60, "HYPERLAPSE"},  /* 移动延时                              */
    { 0x06, 0x60, "SLOWMO"    },  /* 慢动作      2.7K|100|DEW（100fps）    */
    { 0x13, 0x60, "FREEVIDEO" },  /* 自由比例录像 与录像同规格 4K|30|UW     */
    /* ---- 照片类（payload[4]=0x70）---- */
    { 0x01, 0x70, "PHOTO"     },  /* 拍照                                  */
    { 0x02, 0x70, "HDR PHOTO" },  /* HDR拍照                               */
    { 0x03, 0x70, "INTV PHOTO"},  /* 间隔拍照    规格 16:9|3s|MEGA          */
    { 0x05, 0x70, "STARLAPSE" },  /* 星空延时                              */
};

static const char *insta360_mode_name_from_code(uint8_t code3, uint8_t code4)
{
    for (size_t i = 0; i < sizeof(INSTA360_MODES) / sizeof(INSTA360_MODES[0]); i++) {
        if (INSTA360_MODES[i].code3 == code3 && INSTA360_MODES[i].code4 == code4) {
            return INSTA360_MODES[i].name;
        }
    }
    return NULL;
}
static TimerHandle_t s_recording_timer = NULL;

/* Advertising. */
static bool s_adv_pending = false;
static bool s_scan_rsp_pending = false;
static uint8_t s_wake_adv_data[64];
static size_t s_wake_adv_len = 0;

/* Wake payload (last 6 chars of camera BLE name), NVS-backed. */
static char s_wake_payload[6] = {'1', '2', '3', '4', '5', '6'};
static bool s_payload_is_default = true;
static bool s_scan_active = false;

/* 唤醒 payload / 型号 待落盘标记。
 * 扫描回调跑在 BT 任务里，不能在那里写 flash（会阻塞协议栈、拖垮刚建立的连接），
 * 所以回调里只置位，实际 nvs_commit 交给定时器任务完成。 */
static volatile bool s_nvs_dirty = false;

/* 开机待执行的名称扫描。
 * 不能在 GATTS 注册/广播之前就开扫：扫描与广播共用控制器，会把广播起始一起拖住，
 * 相机十几秒内扫不到本机 → 表现为「连接缓慢」。改为广播真正起来之后再扫。 */
static bool s_pending_name_scan = false;

/* 相机型号（广播名去掉后 6 位 hex），NVS-backed，如 "Ace Pro 2"。 */
static char s_model[32] = "";

static const esp_bt_uuid_t s_write_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = { INSTA360_UUID128(0x81) },
};
static const esp_bt_uuid_t s_notify_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid.uuid128 = { INSTA360_UUID128(0x82) },
};
static const esp_bt_uuid_t s_cccd_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG};

static esp_gatt_srvc_id_t s_service_id = {
    .is_primary = true,
    .id = {
        .inst_id = 0x00,
        .uuid = {
            .len = ESP_UUID_LEN_128,
            .uuid.uuid128 = { INSTA360_UUID128(0x80) },
        },
    },
};

static esp_attr_value_t s_write_char_val = {
    .attr_max_len = 512,
    .attr_len = 0,
    .attr_value = NULL,
};

static esp_attr_value_t s_notify_char_val = {
    .attr_max_len = 512,
    .attr_len = 0,
    .attr_value = NULL,
};

static uint8_t s_cccd_init[2] = {0x00, 0x00};
static esp_attr_value_t s_cccd_val = {
    .attr_max_len = 2,
    .attr_len = 2,
    .attr_value = s_cccd_init,
};

/* 属性应答控制。此前 add_char/add_char_descr 的 control 传 NULL，
 * 内部 memset 清零后 auto_rsp=0=ESP_GATT_RSP_BY_APP，即所有读写都要 app
 * 显式 send_response，但旧代码从未应答 → 相机读特征/写 CCCD 会一直等
 * （表现为「连接蓝牙遥控器」转圈 → 超时断开）。这里显式指定：
 *  - 值特征(CE81/CE82)：RSP_BY_APP，由下方 READ/WRITE 事件手动应答；
 *  - CCCD(2902)：AUTO_RSP，协议栈自动应答并管理通知使能，不回投事件。 */
static esp_attr_control_t s_write_char_ctrl  = { .auto_rsp = ESP_GATT_RSP_BY_APP };
static esp_attr_control_t s_notify_char_ctrl = { .auto_rsp = ESP_GATT_RSP_BY_APP };
static esp_attr_control_t s_cccd_ctrl        = { .auto_rsp = ESP_GATT_AUTO_RSP };

/* 广播数据（≤31 字节）：Flags + 128 位服务 UUID。名字放扫描响应，避免超 31 字节。 */
static uint8_t s_normal_adv_data[] = {
    0x02, 0x01, 0x06,                 // Flags: LE General Disc + BR/EDR not supported
    0x11, 0x07,                       // Complete list of 128-bit service UUIDs (17 = 1 type + 16 data)
    INSTA360_UUID128(0x80),           // 0000ce80-0000-1000-8000-00805f9b34fb
};

/* 扫描响应（≤31 字节）：完整设备名。相机主动扫描时读取（Ace Pro 2 按名字精确匹配）。 */
static uint8_t s_scan_rsp_data[] = {
    0x14, 0x09,                       // Complete local name (20 = 1 + 19)
    'I','n','s','t','a','3','6','0',' ','G','P','S',' ','R','e','m','o','t','e',
};

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_scan_params_t s_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};

static void start_advertising(uint8_t *data, size_t len);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* Build the iBeacon wake advertisement from the configured wake payload. */
static void build_wake_adv_data(void) {
    int idx = 0;
    s_wake_adv_data[idx++] = 0x1B;  // manufacturer data length = 27
    s_wake_adv_data[idx++] = 0xFF;  // manufacturer specific
    s_wake_adv_data[idx++] = 0x4C;  // Apple company ID (low)
    s_wake_adv_data[idx++] = 0x00;  // Apple company ID (high)
    s_wake_adv_data[idx++] = 0x02;  // iBeacon format
    s_wake_adv_data[idx++] = 0x15;  // iBeacon format
    memcpy(&s_wake_adv_data[idx], ORBIT_UUID, sizeof(ORBIT_UUID));
    idx += sizeof(ORBIT_UUID);
    for (int i = 0; i < 6; i++) {
        s_wake_adv_data[idx++] = (uint8_t)s_wake_payload[i];
    }

    /* major/minor 实测为 0/0：官方遥控唤醒成功时这两个字段就是 0，说明相机不依赖它们。
     * （Android SDK 里那套「机型码 + 功能码」是手机 App 当遥控时用的，硬件遥控并不发。）
     * iBeacon 规范用大端字节序。 */
    s_wake_adv_data[idx++] = 0x00;  // major high
    s_wake_adv_data[idx++] = 0x00;  // major low
    s_wake_adv_data[idx++] = 0x00;  // minor high
    s_wake_adv_data[idx++] = 0x00;  // minor low
    s_wake_adv_data[idx++] = 0xE4;  // TX power
    s_wake_adv_data[idx++] = 0x01;  // extra

    /* 名字放扫描响应，不放进广播数据（避免超 31 字节）。 */
    s_wake_adv_len = idx;

    ESP_LOGI(TAG, "Wake adv: id='%.6s'", s_wake_payload);
}

/* Load the wake payload from NVS (if a real one was stored previously). */
static void load_wake_payload(void) {
    nvs_handle_t handle;
    if (nvs_open(INSTA360_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        char buf[7] = {0};
        size_t len = sizeof(buf);
        if (nvs_get_str(handle, INSTA360_NVS_KEY_WAKE, buf, &len) == ESP_OK && len == 7) {
            memcpy(s_wake_payload, buf, 6);
            s_payload_is_default = false;
            ESP_LOGI(TAG, "Loaded wake payload '%.6s' from NVS", s_wake_payload);
        }
        nvs_close(handle);
    }
}

/* Persist the wake payload to NVS. */
static void persist_wake_payload(void) {
    nvs_handle_t handle;
    if (nvs_open(INSTA360_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS to persist wake payload");
        return;
    }
    char buf[7] = {0};
    memcpy(buf, s_wake_payload, 6);
    esp_err_t err = nvs_set_str(handle, INSTA360_NVS_KEY_WAKE, buf);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to persist wake payload: %s", esp_err_to_name(err));
    }
}

/* Load the model name from NVS. */
static void load_model(void) {
    nvs_handle_t handle;
    if (nvs_open(INSTA360_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        size_t len = sizeof(s_model);
        if (nvs_get_str(handle, INSTA360_NVS_KEY_MODEL, s_model, &len) == ESP_OK) {
            ESP_LOGI(TAG, "Loaded model '%s' from NVS", s_model);
        }
        nvs_close(handle);
    }
}

/* Persist the model name to NVS. */
static void persist_model(void) {
    nvs_handle_t handle;
    if (nvs_open(INSTA360_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS to persist model");
        return;
    }
    esp_err_t err = nvs_set_str(handle, INSTA360_NVS_KEY_MODEL, s_model);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to persist model: %s", esp_err_to_name(err));
    }
}

/* 相机 6 字符 ID 的字符集：实测为 "57FHDE" —— 大写字母 + 数字，**不是十六进制**。
 * 早期用 isxdigit() 校验，'H' 被判为非法，整条广播名被丢弃：
 * 型号和 ID 都学不到，OSD 设备名因此退化成 NO CAM（实测相机广播名 "GO 3S 57FHDE"）。 */
static bool id6_chars_valid(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (!isalnum((unsigned char)p[i])) {
            return false;
        }
    }
    return true;
}

/* 相机广播名 = "<型号> <6 字符 ID>"，实测如 "GO 3S 57FHDE"，每约 310ms 广播一次。 */
static bool name_ends_with_id6(const uint8_t *name, uint8_t len) {
    if (len < 6) {
        return false;
    }
    return id6_chars_valid(name + len - 6, 6);
}

/* Case-insensitive prefix match against a non-null-terminated name buffer. */
static bool prefix_ieq(const char *name, uint8_t len, const char *prefix) {
    size_t plen = strlen(prefix);
    if (len < plen) {
        return false;
    }
    for (size_t i = 0; i < plen; i++) {
        if (tolower((unsigned char)name[i]) != tolower((unsigned char)prefix[i])) {
            return false;
        }
    }
    return true;
}

/* Known insta360 camera BLE name prefixes (specific-first so "X4 Air"/"GO 3S"/"ONE RS" win). */
static const char *const INSTA360_MODELS[] = {
    "Ace Pro 2", "Ace Pro", "Ace",
    "X4 Air", "X5", "X4", "X3", "X2",
    "GO 3S", "GO 3", "GO 2",
    "ONE RS", "ONE X2", "ONE X", "ONE R",
};

/* True if the name looks like an insta360 camera: "Insta360 …" brand, or a known model. */
static bool is_insta360_camera_name(const char *name, uint8_t len) {
    if (prefix_ieq(name, len, "Insta360")) {
        return true;
    }
    for (size_t i = 0; i < sizeof(INSTA360_MODELS) / sizeof(INSTA360_MODELS[0]); i++) {
        if (prefix_ieq(name, len, INSTA360_MODELS[i])) {
            return true;
        }
    }
    return false;
}

/* Public: insta360 camera name = known model prefix + trailing 6-char ID. */
bool insta360_is_camera_name(const uint8_t *name, uint8_t len) {
    return name_ends_with_id6(name, len) && is_insta360_camera_name((const char *)name, len);
}

/* Public: extract the trailing 6-hex wake payload from a validated camera name. */
void insta360_extract_wake_payload(const uint8_t *name, uint8_t len, char out[6]) {
    if (name == NULL || out == NULL || len < 6) {
        return;
    }
    for (int i = 0; i < 6; i++) {
        out[i] = (char)name[len - 6 + i];
    }
}

/* 诊断用：本轮扫描看到过多少个「带名字的广播」。
 * 型号学不到时，日志里的名字列表是区分「相机根本没在广播」和「广播名被规则拒掉」
 * 的唯一依据——原先校验不过的名字是静默丢弃的，什么都看不到。 */
static uint16_t s_scan_names_seen = 0;

/* Extract the camera name from a scan result and learn the wake payload. */
static void handle_scan_result(esp_ble_gap_cb_param_t *param) {    if (!s_payload_is_default && s_model[0] != '\0') {
        return;  /* already have payload and model */
    }
    if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
        return;
    }

    /* Adv data and scan response share one buffer: ble_adv[0..adv_data_len] then the rsp. */
    uint8_t *adv = param->scan_rst.ble_adv;
    uint8_t len = 0;
    uint8_t *name = esp_ble_resolve_adv_data_by_type(adv, param->scan_rst.adv_data_len,
                                                     ESP_BLE_AD_TYPE_NAME_CMPL, &len);
    if (name == NULL) {
        name = esp_ble_resolve_adv_data_by_type(adv, param->scan_rst.adv_data_len,
                                                ESP_BLE_AD_TYPE_NAME_SHORT, &len);
    }
    if (name == NULL) {
        name = esp_ble_resolve_adv_data_by_type(adv + param->scan_rst.adv_data_len,
                                                param->scan_rst.scan_rsp_len,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &len);
    }
    if (name == NULL) {
        name = esp_ble_resolve_adv_data_by_type(adv + param->scan_rst.adv_data_len,
                                                param->scan_rst.scan_rsp_len,
                                                ESP_BLE_AD_TYPE_NAME_SHORT, &len);
    }
    if (name == NULL) {
        return;
    }

    s_scan_names_seen++;
    ESP_LOGW(TAG, "[扫描] 设备名 '%.*s' (%u 字节)", (int)len, (const char *)name, (unsigned)len);

    if (!name_ends_with_id6(name, len)) {
        return;
    }
    if (!is_insta360_camera_name((const char *)name, len)) {
        ESP_LOGW(TAG, "[扫描]   ↑ 不是已知型号前缀，跳过");
        return;
    }

    for (int i = 0; i < 6; i++) {
        s_wake_payload[i] = (char)name[len - 6 + i];
    }
    s_payload_is_default = false;
    s_nvs_dirty = true;

    /* 提取型号：去掉末尾 6 位 hex，再 trim 尾部空格。 */
    int model_len = len - 6;
    while (model_len > 0 && name[model_len - 1] == ' ') {
        model_len--;
    }
    if (model_len > 0 && model_len < (int)sizeof(s_model)) {
        memcpy(s_model, name, model_len);
        s_model[model_len] = '\0';
        s_nvs_dirty = true;
    }

    ESP_LOGI(TAG, "Auto-learned camera '%s' (wake '%.6s') from '%.*s'",
             s_model, s_wake_payload, (int)len, (const char *)name);
}

/* Start a one-shot scan to discover the camera name (only when payload is still default). */
static void start_camera_scan(void) {
    if (s_scan_active) {
        return;
    }
    esp_err_t ret = esp_ble_gap_set_scan_params(&s_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "set scan params failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_ble_gap_start_scanning(CAMERA_NAME_SCAN_DURATION);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "start scan failed: %s", esp_err_to_name(ret));
        return;
    }
    s_scan_active = true;
    s_scan_names_seen = 0;
    ESP_LOGI(TAG, "Scanning for insta360 camera name to auto-learn wake payload...");
}

void insta360_logic_gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        if (s_adv_pending) {
            s_adv_pending = false;
            /* 广播数据就绪后，配置扫描响应（设备名），就绪后再真正开始广播。 */
            s_scan_rsp_pending = true;
            esp_err_t ret = esp_ble_gap_config_scan_rsp_data_raw(s_scan_rsp_data, sizeof(s_scan_rsp_data));
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "config scan rsp failed: %s", esp_err_to_name(ret));
                s_scan_rsp_pending = false;
                esp_ble_gap_start_advertising(&s_adv_params);
            }
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        if (s_scan_rsp_pending) {
            s_scan_rsp_pending = false;
            esp_err_t ret = esp_ble_gap_start_advertising(&s_adv_params);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "start advertising failed: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG, "Advertising started");
                /* 广播已起来，现在才安全地补扫相机名（型号/唤醒 payload 学习）。
                 * 相机一旦连上，CONNECT_EVT 会立即停掉这次扫描。 */
                if (s_pending_name_scan) {
                    s_pending_name_scan = false;
                    start_camera_scan();
                }
            }
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        handle_scan_result(param);
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        s_scan_active = false;
        ESP_LOGI(TAG, "Camera name scan finished (看到 %u 个带名字的广播，型号='%s')",
                 (unsigned)s_scan_names_seen, s_model[0] ? s_model : "(未学到)");
        break;
    default:
        break;
    }
}

static void start_advertising(uint8_t *data, size_t len) {
    esp_ble_gap_stop_advertising();
    s_adv_pending = true;
    esp_err_t ret = esp_ble_gap_config_adv_data_raw(data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "config adv data failed: %s", esp_err_to_name(ret));
        s_adv_pending = false;
    }
}

static void start_normal_advertising(void) {
    ESP_LOGI(TAG, "Starting normal advertising");
    start_advertising(s_normal_adv_data, sizeof(s_normal_adv_data));
}

static void start_wake_advertising(void) {
    build_wake_adv_data();
    ESP_LOGI(TAG, "Starting wake advertising");
    start_advertising(s_wake_adv_data, s_wake_adv_len);
}

static void send_command(const uint8_t *cmd, size_t len) {
    if (!s_connected) {
        ESP_LOGW(TAG, "No camera connected, cannot send command");
        return;
    }
    esp_err_t ret = esp_ble_gatts_send_indicate(s_gatts_if, s_conn_id, s_notify_char_handle, len, (uint8_t *)cmd, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "send indicate failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Command sent (%zu bytes)", len);
    }
}

/* 把模式字符串格式化成大疆风格 "XKXX XXXX"：
 * "4K|30|DEW" -> "4K30 DEW"（分辨率+帧率连写，附加用空格）
 * "16:9|MEGA" -> "16:9 MEGA"（画幅 + 附加用空格） */
static void format_mode_str(char *s) {
    char *p1 = strchr(s, '|');
    if (p1 == NULL) {
        return;
    }
    char *p2 = strchr(p1 + 1, '|');

    bool f2_numeric = false;
    if (p2 != NULL) {
        f2_numeric = true;
        for (char *q = p1 + 1; q < p2; q++) {
            if (*q < '0' || *q > '9') {
                f2_numeric = false;
                break;
            }
        }
    }

    if (p2 != NULL && f2_numeric) {
        /* 分辨率 + 帧率连写，去掉第一个 '|'，其余 '|' 换空格 */
        memmove(p1, p1 + 1, strlen(p1 + 1) + 1);
        for (char *q = p1; *q; q++) {
            if (*q == '|') {
                *q = ' ';
            }
        }
    } else {
        for (char *q = s; *q; q++) {
            if (*q == '|') {
                *q = ' ';
            }
        }
    }
}

/* 相机通过 write 特征下发状态包。协议格式：
 *   fe ef fe <type> 80 <len> <data[len]>
 * type:
 *   0x0e = 录制开始
 *   0x0f = 录制停止
 *   0x10 = 状态 + ASCII（录制时间 ".HH:MM:SS" 或模式信息）
 *   0x07 = 设备信息（ASCII，如固件标识）
 *   0x02 = 其他状态（含义待定）
 */
static void handle_camera_write(esp_ble_gatts_cb_param_t *param) {
    const uint8_t *data = param->write.value;
    size_t len = param->write.len;
    if (data == NULL || len < 6) {
        return;
    }

    ESP_LOGI(TAG, "Camera write %d bytes:", (int)len);
    ESP_LOG_BUFFER_HEX(TAG, data, len);

    /* 校验协议头 fe ef fe */
    if (data[0] != 0xFE || data[1] != 0xEF || data[2] != 0xFE) {
        return;
    }
    uint8_t type = data[3];
    uint8_t dlen = data[5];
    if (6 + dlen > len) {
        return;
    }
    const uint8_t *payload = &data[6];
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

    switch (type) {
    case 0x0E:  /* 录制开始 */
        if (!s_recording) {
            s_recording = true;
            s_rec_start_ms = now;
            s_rec_seconds = 0;
            s_last_timer_ms = now;
            ESP_LOGI(TAG, "Recording started (type 0x0E)");
        }
        break;

    case 0x0F:  /* 录制停止 */
        if (s_recording) {
            s_recording = false;
            ESP_LOGI(TAG, "Recording stopped (type 0x0F)");
        }
        break;

    case 0x10:  /* 状态 + ASCII */
        /* 头部 4 字节此前一直被忽略，但 payload[1] 是会变的（0x0a~0x24）。
         * 先原样打出来备查——尚未定位的字段（如电量）可能在其中。 */
        ESP_LOGI(TAG, "[状态10] hdr=%02X %02X %02X %02X  |%.*s|",
                 payload[0], payload[1], payload[2], payload[3],
                 (int)(dlen > 4 ? dlen - 4 : 0), (const char *)&payload[4]);
        /* 录制时间：payload[4]=='.' 且后面是 ".HH:MM:SS" */
        if (dlen >= 13 && payload[4] == 0x2E &&
            payload[7] == 0x3A && payload[10] == 0x3A) {
            int hh = (payload[5] - '0') * 10 + (payload[6] - '0');
            int mm = (payload[8] - '0') * 10 + (payload[9] - '0');
            int ss = (payload[11] - '0') * 10 + (payload[12] - '0');
            s_rec_seconds = (uint32_t)(hh * 3600 + mm * 60 + ss);
            if (!s_recording) {
                s_recording = true;
                s_rec_start_ms = now;
                ESP_LOGI(TAG, "Recording started (timer packet)");
            }
            s_last_timer_ms = now;
        } else if (dlen >= 7 && payload[4] == 0x20) {
            /* 剩余：payload[4]==' '，后接三种格式之一：
             *   "数字m"       → 录像剩余分钟（< 1h）
             *   "数字h数字m"  → 录像剩余小时+分钟（>= 1h）
             *   "数字" / "数字+" → 拍照剩余张数
             *
             * 相机**按当前模式二选一上报**：切模式时只发新格式，不发零值。
             * 所以收到一种必须清掉另一种 —— 否则从拍照切回录像后，陈旧的张数
             * 会一直压过新的分钟数（OSD 优先显示 remain_photos>0）。
             * 实测 insta360.pcapng：| 16m| → | 999+| → | 21m| ，中间没有清零包。 */
            uint32_t num = 0;
            size_t i = 5;
            while (i < dlen && payload[i] >= '0' && payload[i] <= '9') {
                num = num * 10 + (uint32_t)(payload[i] - '0');
                i++;
            }

            if (i < dlen && (payload[i] == 'h' || payload[i] == 'H')) {
                /* 小时 + 分钟 */
                uint32_t hours = num;
                num = 0;
                i++;
                while (i < dlen && payload[i] >= '0' && payload[i] <= '9') {
                    num = num * 10 + (uint32_t)(payload[i] - '0');
                    i++;
                }
                if (i < dlen && (payload[i] == 'm' || payload[i] == 'M')) {
                    s_remain_minutes = hours * 60 + num;
                    s_remain_photos = 0;
                    ESP_LOGI(TAG, "Remaining recording: %lu min", (unsigned long)(hours * 60 + num));
                }
            } else if (i < dlen && (payload[i] == 'm' || payload[i] == 'M')) {
                /* 分钟 */
                s_remain_minutes = num;
                s_remain_photos = 0;
                ESP_LOGI(TAG, "Remaining recording: %lu min", (unsigned long)num);
            } else {
                /* 张数 */
                s_remain_photos = num;
                s_remain_minutes = 0;
                ESP_LOGI(TAG, "Remaining photos: %lu", (unsigned long)num);
            }
        } else if (dlen > 4) {
            /* 模式描述：payload[4..dlen-1] 是 ASCII，如 "4K|30|DEW" */
            size_t slen = dlen - 4;
            if (slen >= sizeof(s_mode_str)) {
                slen = sizeof(s_mode_str) - 1;
            }
            memcpy(s_mode_str, &payload[4], slen);
            s_mode_str[slen] = '\0';
            format_mode_str(s_mode_str);
            ESP_LOGI(TAG, "规格串 -> %s", s_mode_str);
        }
        break;

    case 0x05:  /* 录制状态位：载荷 01 = 录制中，00 = 空闲。
                 * 抓包实测：0x05(01) 紧跟 0x0e（开始录制）之前、0x05(00) 紧跟 0x0f
                 *（停止录制）之后，与起停事件一致。
                 * 这里当作**状态校正源**用：事件包（0x0e/0x0f）可能因丢包漏掉，
                 * 而 0x05 直接给出当前状态，能纠偏。 */
        if (dlen >= 1) {
            bool rec = (payload[0] != 0);
            if (rec != s_recording) {
                ESP_LOGI(TAG, "录制状态 -> %s (type 0x05)", rec ? "录制中" : "空闲");
                s_recording = rec;
                if (rec) {
                    /* 同时刷新计时基准，否则刚置位就会被 5 秒无计时包超时清掉 */
                    s_last_timer_ms = (uint32_t)(esp_timer_get_time() / 1000);
                }
            }
        }
        break;

    case 0x07:  /* 相机身份：6 字节 ASCII 相机 ID（唤醒 payload 的权威来源） */
        /* 相机每次连接后主动推来自己的 6 字符 ID，实测为 "57FHDE"
         * （= 相机广播名 "GO 3S 57FHDE" 的后 6 位 = 唤醒信标 UUID 的尾部）。
         * 这条控制链路是**权威来源**：不依赖相机广播、不受扫描窗口限制，
         * 所以优先用它，扫描只作为型号（名称显示用）的兜底。 */
        if (dlen == 6 && id6_chars_valid(payload, 6)) {
            if (s_payload_is_default || memcmp(s_wake_payload, payload, 6) != 0) {
                memcpy(s_wake_payload, payload, 6);
                s_payload_is_default = false;
                s_nvs_dirty = true;
                ESP_LOGI(TAG, "相机 ID -> '%.6s' (来自 type 0x07)", s_wake_payload);
            }
        } else {
            ESP_LOGI(TAG, "Camera info: %.*s", (int)dlen, (const char *)payload);
        }
        break;

    case 0x02:  /* 周期状态包：5 字节二进制。
                 * 两个变体（按 payload[4] 区分）：
                 *   0x50 → 心跳，内容恒为 00 54 00 XX 50。p1=0x54(84) 疑似电量、
                 *          p3 在 0x0a/0x0b 间随场次翻转，疑似充电状态。
                 *   0x60/0x70 → 模式包，p1/p2 恒为 28 17，p3=模式组、p4=视频/照片类
                 *          （见 INSTA360_MODES）。 */
        memcpy(s_status02, payload, sizeof(s_status02));
        ESP_LOGI(TAG, "[状态02] %02X %02X %02X %02X %02X",
                 payload[0], payload[1], payload[2], payload[3], payload[4]);
        if (payload[4] != 0x50) {
            uint16_t key = (uint16_t)((payload[3] << 8) | payload[4]);
            const char *nm = insta360_mode_name_from_code(payload[3], payload[4]);
            if (key != s_mode_key) {
                s_mode_key = key;
                /* 这里不打印规格串：规格串比本包晚约 0.2 秒到，此刻还是上一个模式的。 */
                ESP_LOGI(TAG, "拍摄模式 -> 0x%02X/0x%02X (%s)",
                         payload[3], payload[4], nm ? nm : "未确认");
            }
            if (nm == NULL) {
                ESP_LOGW(TAG, "[状态02] 未建表模式 0x%02X/0x%02X", payload[3], payload[4]);
            }
        } else {
            /* 心跳变体：payload[3] 只有一个字节，含义取决于是否在充电：
             *
             *   0x0B  → **充电中的固定标记值**。实测：任意电量下放进充电仓/插上
             *           充电器，p3 恒为 0x0B（35% 时从 0x08 跳到 0x0B）。此时相机
             *           **不再上报电量档位**，所以不能拿它当百分比用。
             *   其他  → 电量档位，8 档、每档 12.5%：电量 = (p3 - 5) * 12.5
             *           （p3=5 → 0%，p3=13 → 100%）。
             *           校准三点实测：p3=7→25%、p3=8→37.5%、p3=9→50%。
             */
            if (payload[3] == INSTA360_CHARGING_CODE) {
                if (!s_charging) {
                    s_charging = true;
                    ESP_LOGI(TAG, "充电中 (p3=0x%02X)", payload[3]);
                }
            } else {
                const insta360_batt_level_t *lv = insta360_batt_level(payload[3]);
                if (lv != NULL) {
                    if (s_charging || lv->lo != s_battery_lo || lv->hi != s_battery_hi) {
                        ESP_LOGI(TAG, "电量 -> %u-%u%% (p3=0x%02X)%s",
                                 lv->lo, lv->hi, payload[3], s_charging ? "  已停止充电" : "");
                    }
                    s_battery_lo = lv->lo;
                    s_battery_hi = lv->hi;
                    s_battery_label = lv->label;
                } else {
                    /* 未建表的挡位：不猜，置零（OSD 显示 --），并提示补表 */
                    s_battery_lo = s_battery_hi = 0;
                    s_battery_label = BATT_LABEL_NONE;
                    ESP_LOGW(TAG, "未建表电量挡位 p3=0x%02X", payload[3]);
                }
                s_charging = false;
            }
        }
        break;

    default:
        /* 未识别的包类型：打印整帧而不是静默丢弃。
         * 尚未定位的字段（如电量）很可能在没见过的类型里，原先是直接 break 会漏掉线索。 */
        ESP_LOGW(TAG, "[未知包] type=0x%02X dlen=%u 整帧:", type, (unsigned)dlen);
        ESP_LOG_BUFFER_HEX(TAG, data, len);
        break;
    }
}

static void recording_timeout_timer_cb(TimerHandle_t xTimer) {
    /* 本回调在 FreeRTOS 定时器任务里执行（不是 BT 回调），可以安全写 flash。
     * 扫描期间学到的型号/wake payload 在这里落盘。 */
    if (s_nvs_dirty) {
        s_nvs_dirty = false;
        persist_wake_payload();
        persist_model();
    }

    if (s_recording) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - s_last_timer_ms > 5000) {
            s_recording = false;
            ESP_LOGI(TAG, "Recording stopped (timer packet timeout)");
        }
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        s_gatts_if = gatts_if;
        ESP_LOGI(TAG, "GATTS registered, gatts_if=%d", gatts_if);
        esp_ble_gatts_create_service(gatts_if, &s_service_id, GATTS_NUM_HANDLE);
        break;

    case ESP_GATTS_CREATE_EVT:
        s_service_handle = param->create.service_handle;
        ESP_LOGI(TAG, "Service created, handle=%d", s_service_handle);
        s_add_step = 0;
        esp_ble_gatts_add_char(s_service_handle, (esp_bt_uuid_t *)&s_write_char_uuid,
                               ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_WRITE,
                               &s_write_char_val, &s_write_char_ctrl);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (s_add_step == 0) {
            s_write_char_handle = param->add_char.attr_handle;
            s_add_step = 1;
            ESP_LOGI(TAG, "Write char added, handle=%d", s_write_char_handle);
            esp_ble_gatts_add_char(s_service_handle, (esp_bt_uuid_t *)&s_notify_char_uuid,
                                   ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   &s_notify_char_val, &s_notify_char_ctrl);
        } else if (s_add_step == 1) {
            s_notify_char_handle = param->add_char.attr_handle;
            s_add_step = 2;
            ESP_LOGI(TAG, "Notify char added, handle=%d", s_notify_char_handle);
            esp_ble_gatts_add_char_descr(s_service_handle, (esp_bt_uuid_t *)&s_cccd_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                         &s_cccd_val, &s_cccd_ctrl);
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(TAG, "CCCD added, handle=%d", param->add_char_descr.attr_handle);
        esp_ble_gatts_start_service(s_service_handle);
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "Service started");
        start_normal_advertising();
        break;

    case ESP_GATTS_CONNECT_EVT:
        s_conn_id = param->connect.conn_id;
        s_connected = true;
        s_recording = false;

        /* 连接期间绝不扫描：扫描与连接共用射频，会让相机的 GATT 交互超时，
         * 表现为「模块绿灯已亮、相机仍在转圈」。型号/唤醒 payload 的学习只在
         * 开机尚未连接时（insta360_logic_init）做，不在这里补扫。 */
        if (s_scan_active) {
            esp_ble_gap_stop_scanning();
            s_scan_active = false;
        }
        ESP_LOGI(TAG, "Camera connected, conn_id=%d", s_conn_id);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        s_connected = false;
        s_recording = false;
        s_mode_key = 0xFFFF;   /* 模式失效，避免下次连接时显示上一次的模式 */
        s_battery_lo = s_battery_hi = 0;   /* 电量同理，等新连接的心跳包刷新 */
        s_battery_label = BATT_LABEL_NONE;
        s_charging = false;
        ESP_LOGI(TAG, "Camera disconnected");
        start_normal_advertising();
        break;

    case ESP_GATTS_READ_EVT:
        /* 相机连上后会读特征值。RSP_BY_APP 下必须回包，否则相机一直等 → 转圈。
         * 特征暂无有效值，返回 0 字节 + 成功即可解除阻塞。 */
        if (param->read.need_rsp) {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(rsp));
            rsp.attr_value.handle = param->read.handle;
            esp_err_t r = esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                                      param->read.trans_id, ESP_GATT_OK, &rsp);
            ESP_LOGI(TAG, "READ replied (handle=%d) %s", param->read.handle, esp_err_to_name(r));
        }
        break;

    case ESP_GATTS_WRITE_EVT:
        /* 只处理写特征(CE81)；CCCD 已设 AUTO_RSP，由协议栈自动应答并置通知使能。 */
        if (param->write.handle == s_write_char_handle) {
            handle_camera_write(param);
            if (param->write.need_rsp) {
                esp_err_t r = esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                                          param->write.trans_id, ESP_GATT_OK, NULL);
                ESP_LOGI(TAG, "WRITE replied (handle=%d, len=%d) %s",
                         param->write.handle, param->write.len, esp_err_to_name(r));
            }
        }
        break;

    default:
        break;
    }
}

int insta360_logic_init(bool do_name_scan) {
    /* BLE 栈已由 ble_stack_init() 统一拉起，这里只做 GATTS 从机角色初始化。 */

    /* 唤醒 payload / 型号：优先读 NVS。型号未学则登记一次名称扫描，
     * 但**推迟到广播真正起来之后**再执行（见 GAP 回调里的广播成功分支），
     * 否则扫描会把广播起始一起拖住，导致相机迟迟扫不到本机。 */
    load_wake_payload();
    load_model();
    if (do_name_scan || s_model[0] == '\0') {
        s_pending_name_scan = true;
    }

    esp_err_t ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gatts register failed: %s", esp_err_to_name(ret));
        return -1;
    }

    ret = esp_ble_gatts_app_register(0);
    if (ret) {
        ESP_LOGE(TAG, "gatts app register failed: %s", esp_err_to_name(ret));
        return -1;
    }

    s_recording_timer = xTimerCreate("rec_timeout", pdMS_TO_TICKS(1000), pdTRUE, NULL, recording_timeout_timer_cb);
    if (s_recording_timer) {
        xTimerStart(s_recording_timer, 0);
    } else {
        ESP_LOGW(TAG, "Failed to create recording timeout timer");
    }

    ESP_LOGI(TAG, "insta360 logic init success");
    return 0;
}

void insta360_logic_stop(void) {
    esp_ble_gap_stop_advertising();
    if (s_scan_active) {
        esp_ble_gap_stop_scanning();
        s_scan_active = false;
    }

    if (s_gatts_if != ESP_GATT_IF_NONE) {
        esp_ble_gatts_app_unregister(s_gatts_if);
    }
    s_gatts_if = ESP_GATT_IF_NONE;
    s_service_handle = 0;
    s_write_char_handle = 0;
    s_notify_char_handle = 0;
    s_add_step = 0;
    s_connected = false;
    s_recording = false;
    s_adv_pending = false;
    s_scan_rsp_pending = false;

    if (s_recording_timer != NULL) {
        xTimerStop(s_recording_timer, 0);
        xTimerDelete(s_recording_timer, 0);
        s_recording_timer = NULL;
    }

    ESP_LOGI(TAG, "insta360 logic stopped");
}

void insta360_logic_shutter(void) {
    /* 相机不在线：改成广播唤醒信标把它叫回来，而不是静默丢弃按键。
     * 这正是官方遥控在相机关机时的行为（insta4 实证：信标数秒后相机连回）。 */
    if (!s_connected) {
        ESP_LOGI(TAG, "Shutter with no camera connected -> waking via beacon");
        insta360_logic_wake();
        return;
    }
    send_command(SHUTTER_CMD, sizeof(SHUTTER_CMD));
}

/* 切换「相机端」预设/模式：连续发出「选下一个模式」+「确认」两条命令，不等待。
 *
 * 相机侧的模式切换天然是两步（0x01 只进入预切换、0x02 才确认，缺一不可），
 * 但两条命令连续发出时，对用户就是「按一下直接切到下一个模式」，
 * 没有预选停留、也没有等待。
 *
 * 切到哪个模式仍由相机内部的快速切换列表决定，遥控无法指定。
 * 若实测连续发出不稳定（0x02 被当成快门），再考虑在两条之间加小延时。 */
void insta360_logic_mode_next(void) {
    send_command(MODE_NEXT_CMD, sizeof(MODE_NEXT_CMD));
    send_command(SHUTTER_CMD, sizeof(SHUTTER_CMD));
}

/* 睡眠/唤醒切换（链路保持）。同一个码，由相机当前状态决定是睡还是醒，
 * 遥控无需知道状态 —— 按一次切一次（insta3 对照抓包实证）。 */
void insta360_logic_sleep_wake(void) {
    send_command(SLEEP_WAKE_CMD, sizeof(SLEEP_WAKE_CMD));
}

/* 关机：相机会主动断开 BLE 链路（实测发出后约 0.8 秒收到 LL_TERMINATE_IND）。 */
void insta360_logic_power_off(void) {
    send_command(POWER_OFF_CMD, sizeof(POWER_OFF_CMD));
}

/* 深度唤醒：相机已关机、链路已断，只能靠广播 iBeacon 把它叫醒。
 * 信标是**可连接的 ADV_IND**，相机收到后直接连回本机（insta4 实证）。
 * 官方遥控会一直发到相机连上为止（实测 6 秒和 11 秒两段），这里同样以「连上」
 * 为终止条件，15 秒上限兜底 —— 固定发 3 秒可能落在相机的扫描窗口之外。 */
void insta360_logic_wake(void) {
    /* 唤醒要阻塞到相机连上（最长 15 秒）。相机不在时用户可能连按，这里做重入保护：
     * 已经在发信标就直接返回，避免把控制队列堵上一串 15 秒。 */
    if (s_waking) {
        ESP_LOGI(TAG, "Wake already in progress, ignore");
        return;
    }
    s_waking = true;

    ESP_LOGI(TAG, "Sending wake advertisement...");
    start_wake_advertising();
    for (int i = 0; i < 150 && !s_connected; i++) {   /* 150 × 100ms = 15s 上限 */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    start_normal_advertising();

    s_waking = false;
    ESP_LOGI(TAG, "Wake signal sent (%s)", s_connected ? "camera connected" : "timeout");
}

bool insta360_is_connected(void) {
    return s_connected;
}

bool insta360_is_recording(void) {
    return s_recording;
}

uint32_t insta360_get_recording_seconds(void) {
    if (!s_recording) {
        return 0;
    }
    return s_rec_seconds;
}

const char *insta360_get_mode_str(void) {
    return s_mode_str;
}

/* 当前拍摄模式的 OSD 名称（如 "SLOWMO"/"TIMELAPSE"）；未确认的模式返回 NULL。 */
const char *insta360_get_mode_name(void) {
    if (s_mode_key == 0xFFFF) {
        return NULL;
    }
    return insta360_mode_name_from_code((uint8_t)(s_mode_key >> 8), (uint8_t)(s_mode_key & 0xFF));
}

uint32_t insta360_get_remain_minutes(void) {
    return s_remain_minutes;
}

uint32_t insta360_get_remain_photos(void) {
    return s_remain_photos;
}

uint8_t insta360_get_battery_pct(void) {
    return s_battery_lo;
}

/* 电量区间上界（与 get_battery_pct 构成完整区间）。无区间信息时为 0。 */
uint8_t insta360_get_battery_hi(void) {
    return s_battery_hi;
}

/* 电量文字档位（FULL/HIGH/MEDIUM/LOW）；无档位信息时为 BATT_LABEL_NONE。 */
battery_label_t insta360_get_battery_label(void) {
    return s_battery_label;
}

/* 相机是否正在充电（充电时相机不上报电量档位，只发固定标记值）。 */
bool insta360_is_charging(void) {
    return s_charging;
}

const char *insta360_get_model(void) {
    return s_model;
}

/* 型号 → OSD 缩写：GO 3S -> G3S，X5 -> X5，Ace Pro 2 -> ACP2。 */
const char *insta360_model_abbr(const char *model) {
    if (model == NULL) {
        return NULL;
    }
    size_t mlen = strlen(model);
    model = strip_brand_prefix(model, &mlen);
    if (mlen == 0) {
        return NULL;
    }

    /* Ace 系列 */
    if (prefix_ieq(model, mlen, "Ace Pro 2")) return "ACP2";
    if (prefix_ieq(model, mlen, "Ace Pro"))   return "ACP";
    if (prefix_ieq(model, mlen, "Ace"))       return "ACE";

    /* X 系列 */
    if (prefix_ieq(model, mlen, "X4 Air")) return "X4";
    if (prefix_ieq(model, mlen, "X5"))     return "X5";
    if (prefix_ieq(model, mlen, "X4"))     return "X4";
    if (prefix_ieq(model, mlen, "X3"))     return "X3";
    if (prefix_ieq(model, mlen, "X2"))     return "X2";

    /* GO 系列 */
    if (prefix_ieq(model, mlen, "GO 3S")) return "G3S";
    if (prefix_ieq(model, mlen, "GO 3"))  return "G3";
    if (prefix_ieq(model, mlen, "GO 2"))  return "G2";

    return NULL;
}

/* 启动一次相机名扫描（学习型号 + wake payload），供对频时调用。 */
void insta360_logic_start_name_scan(void) {
    start_camera_scan();
}

void insta360_logic_set_wake_payload(const char last6[6]) {
    if (last6 == NULL) {
        return;
    }
    memcpy(s_wake_payload, last6, 6);
    s_payload_is_default = false;
    persist_wake_payload();
    ESP_LOGI(TAG, "Wake payload set to '%.6s'", s_wake_payload);
}
