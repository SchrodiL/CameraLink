/* SPDX-License-Identifier: MIT */

#include "profile.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_util.h"

static const char *TAG = "PROFILE";

#define PROFILE_NVS_NAMESPACE "profile"
#define PROFILE_NVS_KEY_ACTIVE "active"
#define PROFILE_NVS_KEY_PREFIX "profile_"

static uint8_t s_active = 0;

static void profile_key(char key[16], uint8_t idx) {
    snprintf(key, 16, "%s%u", PROFILE_NVS_KEY_PREFIX, (unsigned)idx);
}

void profile_init(void) {
    s_active = 0;
    if (nvs_ensure_ready() == ESP_OK) {
        nvs_handle_t handle;
        if (nvs_open(PROFILE_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
            uint8_t v = 0;
            if (nvs_get_u8(handle, PROFILE_NVS_KEY_ACTIVE, &v) == ESP_OK && v < PROFILE_COUNT) {
                s_active = v;
            }
            nvs_close(handle);
        }
    }
    ESP_LOGI(TAG, "profile active = %u", (unsigned)s_active);
}

uint8_t profile_get_active(void) {
    return s_active;
}

void profile_from_live(profile_t *out) {
    osd_config_get_slots(out->slots);
    out->batt_alarm = osd_config_get_batt_alarm();
    channel_map_get_bindings(out->bindings);
}

void profile_set_active(uint8_t idx) {
    if (idx >= PROFILE_COUNT) return;

    profile_t p;
    if (!profile_load(idx, &p)) {
        ESP_LOGW(TAG, "profile %u empty, skip apply", (unsigned)idx);
        return;
    }

    osd_config_set_slots(p.slots);
    osd_config_set_batt_alarm(p.batt_alarm);
    channel_map_set_bindings(p.bindings);

    s_active = idx;
    if (nvs_ensure_ready() == ESP_OK) {
        nvs_handle_t handle;
        if (nvs_open(PROFILE_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
            esp_err_t err = nvs_set_u8(handle, PROFILE_NVS_KEY_ACTIVE, idx);
            if (err == ESP_OK) err = nvs_commit(handle);
            nvs_close(handle);
        }
    }
    ESP_LOGI(TAG, "activated profile %u", (unsigned)idx);
}

void profile_save(uint8_t idx, const profile_t *p) {
    if (idx >= PROFILE_COUNT) return;

    if (nvs_ensure_ready() != ESP_OK) {
        ESP_LOGE(TAG, "nvs init failed");
        return;
    }
    nvs_handle_t handle;
    if (nvs_open(PROFILE_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "open nvs failed");
        return;
    }
    char key[16];
    profile_key(key, idx);
    esp_err_t err = nvs_set_blob(handle, key, p, sizeof(*p));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) ESP_LOGE(TAG, "save profile %u failed: %s", (unsigned)idx, esp_err_to_name(err));
}

bool profile_load(uint8_t idx, profile_t *out) {
    if (idx >= PROFILE_COUNT) return false;
    if (nvs_ensure_ready() != ESP_OK) return false;

    nvs_handle_t handle;
    if (nvs_open(PROFILE_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return false;

    char key[16];
    profile_key(key, idx);
    size_t len = sizeof(*out);
    esp_err_t err = nvs_get_blob(handle, key, out, &len);
    nvs_close(handle);

    return (err == ESP_OK && len == sizeof(*out));
}
