/* SPDX-License-Identifier: MIT */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_gap_ble_api.h"

#include "pairing.h"
#include "camera_backend.h"

static const char *TAG = "PAIRING";

static bool s_active = false;
static bool s_prev_connected = false;          /* 边沿检测基准：上一轮是否连接 */
static backend_id_t s_prev_id = BACKEND_DJI;   /* 进入对频前的后端，取消时恢复 */

/* 后端检测结果待应用标记（由 pairing_poll 在任务上下文里消费）。 */
static bool s_detect_pending = false;
static backend_id_t s_detected = BACKEND_DJI;

/* 对频前关闭 WiFi 的回调（web_server 在 init 时注册）。 */
static void (*s_wifi_off_cb)(void) = NULL;

static esp_ble_scan_params_t s_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};

/* 对频轮询任务：独立于控制队列运行，避免控制队列繁忙时对频检测被饿死。 */
static void pairing_task(void *arg)
{
    (void)arg;
    for (;;) {
        pairing_poll();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

int pairing_init(void)
{
    if (xTaskCreate(pairing_task, "pairing", 3072, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create pairing task");
        return -1;
    }
    return 0;
}

bool pairing_is_active(void)
{
    return s_active;
}

void pairing_set_wifi_off_cb(void (*cb)(void))
{
    s_wifi_off_cb = cb;
}

void pairing_notify_detected(backend_id_t id)
{
    pairing_exit();   /* 退出对频状态（停止扫描） */
    s_detected = id;
    s_detect_pending = true;
}

void pairing_enter(void)
{
    if (s_active) {
        /* 已在对频：长按 = 取消对频，回到进入对频前的后端角色。 */
        ESP_LOGI(TAG, "Cancel pairing, restoring previous backend");
        pairing_exit();
        camera_backend_run_role(s_prev_id);
        return;
    }

    s_prev_id = camera_backend_active_id();

    /* 对频需要 BLE 独占射频，退出 WiFi 避免共存干扰。 */
    if (s_wifi_off_cb != NULL) {
        s_wifi_off_cb();
    }

    /* 先置位：停当前角色触发的 GAP 事件要路由到本模块而不是旧模块。 */
    s_active = true;

    camera_backend_stop_current();
    s_prev_connected = camera_backend_active() != NULL ? camera_backend_active()->is_connected() : false;

    if (s_prev_id == BACKEND_INSTA360) {
        /* insta360 对频：广播等相机连（相机主动连遥控器）。 */
        camera_backend_run_role(BACKEND_INSTA360);
        ESP_LOGI(TAG, "Pairing insta360 (advertising, waiting for camera)...");
    } else {
        /* DJI 对频：扫描 DJI 相机（遥控器主动连相机），扫描检测由 DJI 后端处理。 */
        esp_err_t ret = esp_ble_gap_set_scan_params(&s_scan_params);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "set scan params failed: %s", esp_err_to_name(ret));
            s_active = false;
            camera_backend_run_role(s_prev_id);
            return;
        }
        ESP_LOGI(TAG, "Pairing DJI (scanning)...");
    }
}

void pairing_exit(void)
{
    if (!s_active) {
        return;
    }
    s_active = false;
    esp_ble_gap_stop_scanning();
}

void pairing_poll(void)
{
    /* insta360 对频：从「断开变为连接」判定为检测到。 */
    const camera_backend_t *be = camera_backend_active();
    bool connected = (be != NULL && be->is_connected != NULL) ? be->is_connected() : false;
    if (pairing_is_active() && connected && !s_prev_connected) {
        ESP_LOGI(TAG, "insta360 camera connected during pairing");
        pairing_exit();
        camera_backend_switch_to(camera_backend_active_id());
    }
    s_prev_connected = connected;

    /* 应用后端检测结果（在任务上下文里切换，避免从 BLE 回调重入）。 */
    if (s_detect_pending) {
        s_detect_pending = false;
        camera_backend_switch_to(s_detected);
    }
}
