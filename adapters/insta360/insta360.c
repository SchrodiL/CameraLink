/*
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

/* 9-byte command payloads. cmd byte[7]: 02=shutter, 03=power off. */
static const uint8_t SHUTTER_CMD[]    = {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x02, 0x00};
static const uint8_t POWER_OFF_CMD[]  = {0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x00, 0x03};

/* 10-byte Insta360 iBeacon UUID prefix ("ORBIT"). */
static const uint8_t ORBIT_UUID[10] = {0x09, 0x4F, 0x52, 0x42, 0x49, 0x54, 0x09, 0xFF, 0x0F, 0x00};

/* GATT server state. */
static uint16_t s_gatts_if = ESP_GATT_IF_NONE;
static uint16_t s_service_handle = 0;
static uint16_t s_write_char_handle = 0;
static uint16_t s_notify_char_handle = 0;
static uint8_t s_add_step = 0;  // 0=idle, 1=write char added, 2=notify char added

/* Connection / recording state. */
static bool s_connected = false;
static uint16_t s_conn_id = 0;
static bool s_recording = false;
static uint32_t s_last_timer_ms = 0;
static uint32_t s_rec_start_ms = 0;
static uint32_t s_rec_seconds = 0;   /* 相机下发的精确录制秒数（.HH:MM:SS 解析而来） */
static char s_mode_str[32] = "";     /* 相机下发的模式描述，如 "4K|30|DEW" */
static uint32_t s_remain_minutes = 0; /* 剩余录制时长（分钟），如 "22m" */
static uint32_t s_remain_photos = 0;  /* 拍照模式剩余张数，如 "837" */
static uint8_t s_battery_pct = 0;     /* 电量百分比 0-100（type 0x02 的 data[1]） */
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
    s_wake_adv_data[idx++] = 0x00;  // major high
    s_wake_adv_data[idx++] = 0x00;  // major low
    s_wake_adv_data[idx++] = 0x00;  // minor high
    s_wake_adv_data[idx++] = 0x00;  // minor low
    s_wake_adv_data[idx++] = 0xE4;  // TX power
    s_wake_adv_data[idx++] = 0x01;  // extra

    /* 名字放扫描响应，不放进广播数据（避免超 31 字节）。 */
    s_wake_adv_len = idx;
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

/* True if the last 6 chars of the name are all hex digits (insta360 camera-ID convention). */
static bool name_ends_with_hex6(const uint8_t *name, uint8_t len) {
    if (len < 6) {
        return false;
    }
    for (uint8_t i = len - 6; i < len; i++) {
        if (!isxdigit((unsigned char)name[i])) {
            return false;
        }
    }
    return true;
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

/* Public: insta360 camera name = known model prefix + trailing 6 hex chars. */
bool insta360_is_camera_name(const uint8_t *name, uint8_t len) {
    return name_ends_with_hex6(name, len) && is_insta360_camera_name((const char *)name, len);
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

/* Extract the camera name from a scan result and learn the wake payload. */
static void handle_scan_result(esp_ble_gap_cb_param_t *param) {
    if (!s_payload_is_default && s_model[0] != '\0') {
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
    if (name == NULL || !name_ends_with_hex6(name, len)) {
        return;
    }
    if (!is_insta360_camera_name((const char *)name, len)) {
        ESP_LOGW(TAG, "Skipping '%.*s': hex suffix but not a known insta360 model",
                 (int)len, (const char *)name);
        return;
    }

    for (int i = 0; i < 6; i++) {
        s_wake_payload[i] = (char)name[len - 6 + i];
    }
    s_payload_is_default = false;
    persist_wake_payload();

    /* 提取型号：去掉末尾 6 位 hex，再 trim 尾部空格。 */
    int model_len = len - 6;
    while (model_len > 0 && name[model_len - 1] == ' ') {
        model_len--;
    }
    if (model_len > 0 && model_len < (int)sizeof(s_model)) {
        memcpy(s_model, name, model_len);
        s_model[model_len] = '\0';
        persist_model();
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
            }
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        handle_scan_result(param);
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        s_scan_active = false;
        ESP_LOGI(TAG, "Camera name scan finished");
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
             *   "数字" / "数字+" → 拍照剩余张数 */
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
                    ESP_LOGI(TAG, "Remaining recording: %lu min", (unsigned long)(hours * 60 + num));
                }
            } else if (i < dlen && (payload[i] == 'm' || payload[i] == 'M')) {
                /* 分钟 */
                s_remain_minutes = num;
                ESP_LOGI(TAG, "Remaining recording: %lu min", (unsigned long)num);
            } else {
                /* 张数 */
                s_remain_photos = num;
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
            ESP_LOGI(TAG, "Camera mode: %s", s_mode_str);
        }
        break;

    case 0x07:  /* 设备信息 */
        ESP_LOGI(TAG, "Camera info: %.*s", (int)dlen, (const char *)payload);
        break;

    case 0x02:  /* 状态包：字段含义待定（data[2]=录制状态 0x51/0x17） */
        ESP_LOGI(TAG, "State 0x02: d0=%u d1=%u d2=%u d3=%u d4=%u",
                 (unsigned)payload[0], (unsigned)payload[1], (unsigned)payload[2],
                 (unsigned)payload[3], (unsigned)payload[4]);
        break;

    default:
        break;
    }
}

static void recording_timeout_timer_cb(TimerHandle_t xTimer) {
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
        ESP_LOGI(TAG, "Camera connected, conn_id=%d", s_conn_id);
        /* 型号还没学到就重新扫相机广播名（相机连上时肯定在广播，能拿到型号）。 */
        if (s_model[0] == '\0') {
            start_camera_scan();
        }
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        s_connected = false;
        s_recording = false;
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

    /* 唤醒 payload / 型号：优先读 NVS。型号未学就扫描广播名学习（不依赖对频）。 */
    load_wake_payload();
    load_model();
    if (do_name_scan || s_model[0] == '\0') {
        start_camera_scan();
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
    send_command(SHUTTER_CMD, sizeof(SHUTTER_CMD));
}

void insta360_logic_sleep(void) {
    send_command(POWER_OFF_CMD, sizeof(POWER_OFF_CMD));
}

void insta360_logic_wake(void) {
    ESP_LOGI(TAG, "Sending wake advertisement...");
    start_wake_advertising();
    vTaskDelay(pdMS_TO_TICKS(3000));
    start_normal_advertising();
    ESP_LOGI(TAG, "Wake signal sent");
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

uint32_t insta360_get_remain_minutes(void) {
    return s_remain_minutes;
}

uint32_t insta360_get_remain_photos(void) {
    return s_remain_photos;
}

uint8_t insta360_get_battery_pct(void) {
    return s_battery_pct;
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
