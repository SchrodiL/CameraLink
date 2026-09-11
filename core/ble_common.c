/* SPDX-License-Identifier: MIT */

#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "nvs_util.h"

#include "ble_common.h"
#include "camera_backend.h"
#include "pairing.h"
#include "insta360.h"
#include "dji_ble.h"

static const char *TAG = "BLE_COMMON";

/* 统一 GAP 回调：对频中 → 路由到活动后端的对频 GAP 处理，否则按当前后端转发。 */
static void ble_stack_gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (pairing_is_active()) {
        /* 对频阶段：DJI 后端扫描检测相机 / insta360 后端处理广播事件。 */
        const camera_backend_t *be = camera_backend_active();
        if (be != NULL && be->pairing_gap_handler != NULL) {
            be->pairing_gap_handler(event, param);
        }
        return;
    }
    if (camera_backend_active_id() == BACKEND_INSTA360) {
        insta360_logic_gap_handler(event, param);
    } else {
        ble_client_gap_handler(event, param);
    }
}

int ble_stack_init(void) {
    if (nvs_ensure_ready() != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed");
        return -1;
    }

    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "controller init failed: %s", esp_err_to_name(ret));
        return -1;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "controller enable failed: %s", esp_err_to_name(ret));
        return -1;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "bluedroid init failed: %s", esp_err_to_name(ret));
        return -1;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "bluedroid enable failed: %s", esp_err_to_name(ret));
        return -1;
    }

    /* 本地 MTU 提升到 500：默认 23 字节会让相机下发的状态串/协议包被碎片化，
     * 既慢又容易在弱信号下丢包。原先只在 DJI 角色的 ble_init() 里设置，
     * insta360 角色（GATTS 从机）走不到那里，所以统一放到栈初始化里。 */
    esp_ble_gatt_set_local_mtu(500);

    ret = esp_ble_gap_register_callback(ble_stack_gap_handler);
    if (ret) {
        ESP_LOGE(TAG, "gap register failed: %s", esp_err_to_name(ret));
        return -1;
    }

    ESP_LOGI(TAG, "BLE stack init success");
    return 0;
}
