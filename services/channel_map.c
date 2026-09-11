/* SPDX-License-Identifier: MIT */

#include "channel_map.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_util.h"

#include "camera_backend.h"
#include "controller.h"
#include "pairing.h"

static const char *TAG = "CHMAP";

#define CHMAP_NVS_NAMESPACE "chmap"
/* 键名带版本：功能枚举一旦增删，旧配置各元素的索引含义就变了。换键名让旧配置
 * 自然失效、回落到默认（全禁用），而不是被按新索引**错位解读**
 * （那会把「对频」当成「协议切换」触发，比丢掉配置危险得多）。 */
#define CHMAP_NVS_KEY       "bindings_v2"

static channel_binding_t s_bindings[CHAN_FUNC_COUNT];
static uint16_t s_last_channels[16] = { 0 };
static SemaphoreHandle_t s_mutex = NULL;
static bool s_loaded = false;

/* 默认全部禁用：channel=0, min=0, max=0 */
static void reset_defaults(channel_binding_t b[CHAN_FUNC_COUNT]) {
    memset(b, 0, sizeof(channel_binding_t) * CHAN_FUNC_COUNT);
}

void channel_map_init(void) {
    reset_defaults(s_bindings);

    if (nvs_ensure_ready() == ESP_OK) {
        nvs_handle_t handle;
        if (nvs_open(CHMAP_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
            channel_binding_t buf[CHAN_FUNC_COUNT];
            size_t len = sizeof(buf);
            /* 接受**较短**的旧 blob：新增功能项追加在枚举末尾，旧固件存的 blob
             * 不包含它们。若这里要求 len 完全相等，新增功能就会把用户已存的配置
             * 整体判为不合法并丢掉 —— 只拷贝实际长度，尾部保持默认（禁用）。 */
            if (nvs_get_blob(handle, CHMAP_NVS_KEY, buf, &len) == ESP_OK &&
                len >= sizeof(channel_binding_t) && len <= sizeof(buf)) {
                memcpy(s_bindings, buf, len);
            }
            nvs_close(handle);
        }
    }

    s_mutex = xSemaphoreCreateMutex();
    s_loaded = true;
    ESP_LOGI(TAG, "channel map initialized");
}

void channel_map_get_bindings(channel_binding_t bindings[CHAN_FUNC_COUNT]) {
    if (s_mutex && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(bindings, s_bindings, sizeof(channel_binding_t) * CHAN_FUNC_COUNT);
        xSemaphoreGive(s_mutex);
    } else if (!s_mutex) {
        memcpy(bindings, s_bindings, sizeof(channel_binding_t) * CHAN_FUNC_COUNT);
    }
}

void channel_map_set_bindings(const channel_binding_t bindings[CHAN_FUNC_COUNT]) {
    channel_binding_t clamped[CHAN_FUNC_COUNT];
    memcpy(clamped, bindings, sizeof(clamped));

    /* 非法值钳制：channel > 16 视为禁用 */
    for (int i = 0; i < CHAN_FUNC_COUNT; i++) {
        if (clamped[i].channel > 16) {
            clamped[i].channel = 0;
            clamped[i].min = 0;
            clamped[i].max = 0;
        }
    }

    if (s_mutex && xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(s_bindings, clamped, sizeof(clamped));
        xSemaphoreGive(s_mutex);
    } else {
        memcpy(s_bindings, clamped, sizeof(clamped));
    }

    if (nvs_ensure_ready() == ESP_OK) {
        nvs_handle_t handle;
        if (nvs_open(CHMAP_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
            esp_err_t err = nvs_set_blob(handle, CHMAP_NVS_KEY, clamped, sizeof(clamped));
            if (err == ESP_OK) err = nvs_commit(handle);
            nvs_close(handle);
            if (err != ESP_OK) ESP_LOGE(TAG, "persist bindings failed: %s", esp_err_to_name(err));
        }
    }
}

/* ---- 功能触发 ---- */

void channel_map_feed(const uint16_t ch[16]) {
    static bool s_last[CHAN_FUNC_COUNT] = { false };

    memcpy(s_last_channels, ch, sizeof(s_last_channels));

    channel_binding_t b[CHAN_FUNC_COUNT];
    channel_map_get_bindings(b);

    for (int f = 1; f < CHAN_FUNC_COUNT; f++) {
        const channel_binding_t *bind = &b[f];
        if (bind->channel == 0 || (bind->min == 0 && bind->max == 0)) {
            s_last[f] = false;
            continue;
        }
        if (bind->channel > 16) {
            s_last[f] = false;
            continue;
        }

        uint16_t v = ch[bind->channel - 1];
        bool active = (v >= bind->min && v <= bind->max);
        bool rising = active && !s_last[f];
        s_last[f] = active;

        /* 全部功能都是边沿触发：进范围触发一次。 */
        if (!rising) {
            continue;
        }

        switch ((channel_func_t)f) {
        case CHAN_FUNC_SHUTTER:
            /* 只管「按一下」—— 是开始录、停止录还是拍照，由相机自己决定。
             * 走单机 BOOT 键同一条路径，保证两种触发源行为一致。 */
            controller_single_press();
            break;
        case CHAN_FUNC_PROTO_SWITCH:
            controller_switch_protocol((camera_backend_active_id() == BACKEND_INSTA360) ? BACKEND_DJI : BACKEND_INSTA360);
            break;
        case CHAN_FUNC_PAIRING:
            controller_pairing_enter();
            break;
        case CHAN_FUNC_CAMERA_PRESET:
            controller_preset_next();
            break;
        case CHAN_FUNC_SLEEP_WAKE:
            controller_sleep_wake();
            break;
        case CHAN_FUNC_POWER_OFF:
            controller_power_off();
            break;
        case CHAN_FUNC_WAKE_BEACON:
            controller_wake_beacon();
            break;
        default:
            break;
        }
    }
}

void channel_map_get_last_channels(uint16_t ch[16]) {
    memcpy(ch, s_last_channels, sizeof(s_last_channels));
}
