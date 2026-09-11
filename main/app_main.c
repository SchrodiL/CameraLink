/* SPDX-License-Identifier: MIT
 * Copyright (c) 2025 SZ DJI Technology Co., Ltd.
 * Source: dji-sdk/Osmo-GPS-Controller-Demo (MIT)
 */

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "camera_backend.h"
#include "controller.h"
#include "gps.h"
#include "key.h"
#include "light.h"
#include "osd.h"
#include "ble_common.h"
#include "pairing.h"
#include "channel_map.h"
#include "web_server.h"
#include "dji_backend.h"
#include "insta360_backend.h"

static const char *TAG = "APP";

/**
 * @brief Main application function, performs initialization and task loop
 * 应用主函数，执行初始化和任务循环
 *
 * This function initializes the RGB light, GPS module, Bluetooth connection, 
 * and key logic in sequence, and starts a loop task for periodic operations.
 * 
 * 在此函数中，依次初始化氛围灯、GPS模块、蓝牙模块和按键逻辑，
 * 并启动一个循环任务，周期性进行操作。
 */
void app_main(void) {

    int res = 0;

    /* Initialize RGB light */
    /* 初始化氛围灯 */
    res = init_light_logic();
    if (res != 0) {
        return;
    }

    /* GPS 任务在两种协议下都启动（insta360 模式也读取 GPS 供 OSD 显示） */
    initSendGpsDataToCameraTask();

    /* 统一拉起 BLE 栈（controller + bluedroid + 统一 GAP 回调），只做一次 */
    /* Bring up the BLE stack once (controller + bluedroid + unified GAP callback). */
    if (ble_stack_init() != 0) {
        return;
    }
    /* 注册后端 + 统一控制工作队列 + 对频编排 */
    dji_backend_register();
    insta360_backend_register();
    if (controller_init() != 0) {
        ESP_LOGE(TAG, "controller init failed");
        return;
    }
    if (pairing_init() != 0) {
        ESP_LOGE(TAG, "pairing init failed");
        return;
    }

    if (!camera_backend_is_paired()) {
        /* 首次上电未对频：自动进入对频扫描，识别到即运行时切换（不阻塞、不重启） */
        /* First boot, never paired: auto-enter pairing scan. */
        pairing_enter();
    } else {
        /* 已对频：直接切到上次保存的协议（运行时启动对应角色，不重启） */
        /* Already paired: switch to the persisted protocol at runtime (no reboot). */
        camera_backend_switch_to(camera_backend_active_id());
    }

    /* Initialize key logic */
    /* 初始化按键逻辑 */
    key_logic_init();

    /* 启动 MSP/OSD 任务，把相机状态写入飞控 OSD（两种协议都工作） */
    /* Start MSP/OSD task to write camera state to the FC OSD (both protocols) */
    osd_logic_init();

    /* 通道映射 + WebUI（WiFi AP + HTTP） */
    channel_map_init();
    web_server_init();

    // ===== Subsequent logic loop =====
    // ===== 后续逻辑循环 =====
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
