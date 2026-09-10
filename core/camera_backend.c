/* SPDX-License-Identifier: MIT */

#include "camera_backend.h"

#include <stdbool.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_util.h"

static const char *TAG = "CAM_BACKEND";

#define BACKEND_NVS_NAMESPACE "proto_cfg"
#define BACKEND_NVS_KEY_MODE   "mode"
#define BACKEND_NVS_KEY_PAIRED "paired"

/* 编译期默认协议，仅作「已 paired 但 mode 缺失」的回退。 */
#define BACKEND_DEFAULT_MODE BACKEND_DJI

#define MAX_BACKENDS 4

/* ---- 注册表 ---- */
static const camera_backend_t *s_backends[MAX_BACKENDS];
static size_t s_backend_count = 0;

/* ---- 已决定的协议（RAM 缓存）。对频期间尚未决定，保持上一次的值。 ---- */
static backend_id_t s_mode = BACKEND_DEFAULT_MODE;
static bool s_mode_loaded = false;

/* 当前运行中的角色（insta360 GATTS 从机 / DJI GATTC 主机 / 无）。 */
typedef enum { ROLE_NONE = 0, ROLE_DJI, ROLE_INSTA360 } running_role_t;
static running_role_t s_running_role = ROLE_NONE;

void camera_backend_register(const camera_backend_t *be)
{
    if (be == NULL || s_backend_count >= MAX_BACKENDS) {
        return;
    }
    s_backends[s_backend_count++] = be;
}

const camera_backend_t *camera_backend_get(backend_id_t id)
{
    for (size_t i = 0; i < s_backend_count; i++) {
        if (s_backends[i]->id == id) {
            return s_backends[i];
        }
    }
    return NULL;
}

const camera_backend_t *camera_backend_active(void)
{
    camera_backend_active_id(); /* 确保已从 NVS 载入 */
    return camera_backend_get(s_mode);
}

static running_role_t role_of(backend_id_t id)
{
    return (id == BACKEND_INSTA360) ? ROLE_INSTA360 : ROLE_DJI;
}

backend_id_t camera_backend_active_id(void)
{
    if (s_mode_loaded) {
        return s_mode;
    }

    if (nvs_ensure_ready() != ESP_OK) {
        s_mode = BACKEND_DEFAULT_MODE;
        s_mode_loaded = true;
        return s_mode;
    }

    nvs_handle_t handle;
    if (nvs_open(BACKEND_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        uint8_t val = 0;
        if (nvs_get_u8(handle, BACKEND_NVS_KEY_MODE, &val) == ESP_OK) {
            s_mode = (val == BACKEND_INSTA360) ? BACKEND_INSTA360 : BACKEND_DEFAULT_MODE;
        } else {
            s_mode = BACKEND_DEFAULT_MODE;
        }
        nvs_close(handle);
    } else {
        s_mode = BACKEND_DEFAULT_MODE;
    }

    s_mode_loaded = true;
    ESP_LOGI(TAG, "Active backend: %s", (s_mode == BACKEND_INSTA360) ? "insta360" : "DJI");
    return s_mode;
}

static void persist_mode(backend_id_t mode)
{
    if (nvs_ensure_ready() != ESP_OK) {
        return;
    }
    nvs_handle_t handle;
    if (nvs_open(BACKEND_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    uint8_t val = (mode == BACKEND_INSTA360) ? BACKEND_INSTA360 : BACKEND_DJI;
    esp_err_t err = nvs_set_u8(handle, BACKEND_NVS_KEY_MODE, val);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
}

bool camera_backend_is_paired(void)
{
    if (nvs_ensure_ready() != ESP_OK) {
        return false;
    }
    nvs_handle_t handle;
    if (nvs_open(BACKEND_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    uint8_t val = 0;
    esp_err_t err = nvs_get_u8(handle, BACKEND_NVS_KEY_PAIRED, &val);
    nvs_close(handle);
    return (err == ESP_OK && val != 0);
}

void camera_backend_set_paired(bool paired)
{
    if (nvs_ensure_ready() != ESP_OK) {
        return;
    }
    nvs_handle_t handle;
    if (nvs_open(BACKEND_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    uint8_t val = paired ? 1 : 0;
    esp_err_t err = nvs_set_u8(handle, BACKEND_NVS_KEY_PAIRED, val);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
}

void camera_backend_stop_current(void)
{
    const camera_backend_t *be = NULL;
    if (s_running_role == ROLE_INSTA360) {
        be = camera_backend_get(BACKEND_INSTA360);
    } else if (s_running_role == ROLE_DJI) {
        be = camera_backend_get(BACKEND_DJI);
    }
    if (be != NULL && be->stop != NULL) {
        be->stop();
    }
    s_running_role = ROLE_NONE;
}

void camera_backend_select(backend_id_t id)
{
    backend_id_t target = (id == BACKEND_INSTA360) ? BACKEND_INSTA360 : BACKEND_DJI;
    s_mode = target;
    s_mode_loaded = true;
    persist_mode(target);
    ESP_LOGI(TAG, "Selected backend: %s", (target == BACKEND_INSTA360) ? "insta360" : "DJI");
}

void camera_backend_run_role(backend_id_t id)
{
    running_role_t target = role_of(id);
    if (s_running_role == target) {
        return; /* 目标角色已在运行，不重复起。 */
    }

    camera_backend_stop_current();

    const camera_backend_t *be = camera_backend_get(id);
    if (be == NULL || be->init == NULL) {
        ESP_LOGE(TAG, "Backend %d not registered", id);
        return;
    }

    ESP_LOGI(TAG, "Starting %s role...", (target == ROLE_INSTA360) ? "insta360" : "DJI");
    be->init();
    s_running_role = target;
}

void camera_backend_switch_to(backend_id_t id)
{
    backend_id_t target = (id == BACKEND_INSTA360) ? BACKEND_INSTA360 : BACKEND_DJI;

    /* 跑目标角色：若已运行（例如对频时先起的 insta360 广播，相机已连上）则不重启。 */
    camera_backend_run_role(target);

    s_mode = target;
    s_mode_loaded = true;
    camera_backend_set_paired(true);
    persist_mode(target);
    ESP_LOGI(TAG, "Active backend: %s", (target == BACKEND_INSTA360) ? "insta360" : "DJI");
}
