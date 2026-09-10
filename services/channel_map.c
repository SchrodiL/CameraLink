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
#include "profile.h"

static const char *TAG = "CHMAP";

#define CHMAP_NVS_NAMESPACE "chmap"
#define CHMAP_NVS_KEY       "bindings"

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
            if (nvs_get_blob(handle, CHMAP_NVS_KEY, buf, &len) == ESP_OK &&
                len == sizeof(buf)) {
                memcpy(s_bindings, buf, sizeof(buf));
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
        bool falling = !active && s_last[f];
        s_last[f] = active;

        switch ((channel_func_t)f) {
        case CHAN_FUNC_RECORD:
            if (rising)      controller_record_start();
            else if (falling) controller_record_stop();
            break;
        case CHAN_FUNC_SHUTTER:
            if (rising) controller_shutter();
            break;
        case CHAN_FUNC_PROTO_SWITCH:
            if (rising) controller_switch_protocol((camera_backend_active_id() == BACKEND_INSTA360) ? BACKEND_DJI : BACKEND_INSTA360);
            break;
        case CHAN_FUNC_PAIRING:
            if (rising) controller_pairing_enter();
            break;
        case CHAN_FUNC_PROFILE_1:
        case CHAN_FUNC_PROFILE_2:
        case CHAN_FUNC_PROFILE_3:
            if (rising) profile_set_active(f - CHAN_FUNC_PROFILE_1);
            break;
        default:
            break;
        }
    }
}

void channel_map_get_last_channels(uint16_t ch[16]) {
    memcpy(ch, s_last_channels, sizeof(s_last_channels));
}
