/* SPDX-License-Identifier: MIT */

#include "osd_config.h"

#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_util.h"

static const char *TAG = "OSD_CFG";

#define OSD_NVS_NAMESPACE "osd_cfg"
#define OSD_NVS_KEY_SLOTS "slots"
#define OSD_NVS_KEY_BATT_ALARM "batt_alarm"

/* 默认低电量报警阈值：25%（0 表示关闭报警） */
#define OSD_DEFAULT_BATT_ALARM 25

/* 默认布局：录制状态、相机电量、录制规格、名称+剩余容量。 */
static const osd_item_t s_default_slots[OSD_SLOT_COUNT] = {
    OSD_ITEM_REC,
    OSD_ITEM_BATTERY,
    OSD_ITEM_SPEC,
    OSD_ITEM_NAME_STORAGE,
};

static osd_item_t s_slots[OSD_SLOT_COUNT];
static bool s_loaded = false;

static uint8_t s_batt_alarm = OSD_DEFAULT_BATT_ALARM;
static bool s_batt_alarm_loaded = false;

/* 非法槽位回退到默认值。 */
static void clamp_slots(osd_item_t slots[OSD_SLOT_COUNT]) {
    for (int i = 0; i < OSD_SLOT_COUNT; i++) {
        if (slots[i] >= OSD_ITEM_COUNT) {
            slots[i] = s_default_slots[i];
        }
    }
}

void osd_config_get_slots(osd_item_t slots[OSD_SLOT_COUNT]) {
    if (!s_loaded) {
        memcpy(s_slots, s_default_slots, sizeof(s_slots));

        if (nvs_ensure_ready() == ESP_OK) {
            nvs_handle_t handle;
            if (nvs_open(OSD_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
                osd_item_t buf[OSD_SLOT_COUNT];
                size_t len = sizeof(buf);
                if (nvs_get_blob(handle, OSD_NVS_KEY_SLOTS, buf, &len) == ESP_OK &&
                    len == sizeof(buf)) {
                    memcpy(s_slots, buf, sizeof(s_slots));
                    clamp_slots(s_slots);
                }
                nvs_close(handle);
            }
        }
        s_loaded = true;
    }

    memcpy(slots, s_slots, sizeof(s_slots));
}

void osd_config_set_slots(const osd_item_t slots[OSD_SLOT_COUNT]) {
    osd_item_t clamped[OSD_SLOT_COUNT];
    memcpy(clamped, slots, sizeof(clamped));
    clamp_slots(clamped);

    /* 立即更新内存缓存（OSD 任务下一轮刷新即生效，无需重启）。 */
    memcpy(s_slots, clamped, sizeof(s_slots));
    s_loaded = true;

    if (nvs_ensure_ready() != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed, cannot persist slots");
        return;
    }

    nvs_handle_t handle;
    if (nvs_open(OSD_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
        return;
    }

    esp_err_t err = nvs_set_blob(handle, OSD_NVS_KEY_SLOTS, clamped, sizeof(clamped));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to persist slots: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "OSD slots updated: [%d %d %d %d]",
                 clamped[0], clamped[1], clamped[2], clamped[3]);
    }
}

uint8_t osd_config_get_batt_alarm(void)
{
    if (!s_batt_alarm_loaded) {
        s_batt_alarm = OSD_DEFAULT_BATT_ALARM;

        if (nvs_ensure_ready() == ESP_OK) {
            nvs_handle_t handle;
            if (nvs_open(OSD_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
                uint8_t v;
                if (nvs_get_u8(handle, OSD_NVS_KEY_BATT_ALARM, &v) == ESP_OK && v <= 100) {
                    s_batt_alarm = v;
                }
                nvs_close(handle);
            }
        }
        s_batt_alarm_loaded = true;
    }

    return s_batt_alarm;
}

void osd_config_set_batt_alarm(uint8_t pct)
{
    if (pct > 100) pct = OSD_DEFAULT_BATT_ALARM; /* 非法值回退默认 */

    /* 立即更新内存缓存（OSD 任务下一轮刷新即生效，无需重启）。 */
    s_batt_alarm = pct;
    s_batt_alarm_loaded = true;

    if (nvs_ensure_ready() != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed, cannot persist batt_alarm");
        return;
    }

    nvs_handle_t handle;
    if (nvs_open(OSD_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
        return;
    }

    esp_err_t err = nvs_set_u8(handle, OSD_NVS_KEY_BATT_ALARM, pct);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to persist batt_alarm: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "batt alarm threshold updated: %u%%", (unsigned)pct);
    }
}
