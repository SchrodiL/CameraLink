/* SPDX-License-Identifier: MIT
 * Copyright (c) 2025 SZ DJI Technology Co., Ltd.
 * Source: dji-sdk/Osmo-GPS-Controller-Demo (MIT)
 */

#include <time.h>
#include <string.h>
#include "key.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "camera_backend.h"
#include "controller.h"
#include "pairing.h"
#include "light.h"
#include "web_server.h"

static const char *TAG = "LOGIC_KEY";

// 按键事件变量，用于存储当前按键事件
// Key event variable used to store current key event
static key_event_t current_key_event = KEY_EVENT_NONE;

// 按键状态，标记当前按键是否被按下
// Key state flag indicating whether the key is currently pressed
static bool key_pressed = false;

// 按键按下的起始时间，用于计算按下持续时间
// Start time when key is pressed, used to calculate press duration
static TickType_t key_press_start_time = 0;

// 协议切换按键状态（长按检测，按下=高电平）
// Protocol-switch button state (long-press detection, active-high)
static bool s_proto_switch_pressed = false;
static TickType_t s_proto_switch_press_start = 0;
static bool s_proto_switch_triggered = false;

// 双键同时按下状态（同时长按 = 开关 WiFi）
// Both-buttons-pressed state (simultaneous long press = toggle WiFi)
static bool s_both_pressed = false;
static TickType_t s_both_press_start = 0;
static bool s_both_triggered = false;

// 长按阈值（例如：按下超过 1 秒认为是长按）
// Long press threshold (e.g., press longer than 1 second is considered long press)
#define LONG_PRESS_THRESHOLD pdMS_TO_TICKS(1000)

// 单击与长按事件检测时间间隔（例如：50ms）
// Time interval for detecting single press and long press events (e.g., 50ms)
#define KEY_SCAN_INTERVAL pdMS_TO_TICKS(50)

/**
 * @brief 处理长按事件
 *        Handle long press event
 *
 * 长按 = 对频（绑定）：进入对频扫描，自动识别附近是 DJI 还是 insta360 相机，
 * 识别到即运行时切换协议（不重启）。
 * Long press = pairing: scan for a nearby DJI/insta360 camera, then switch protocol.
 */
static void handle_boot_long_press() {
    ESP_LOGI(TAG, "Long press: entering pairing mode...");
    controller_pairing_enter();
}

/**
 * @brief 处理协议切换按键
 *        Handle protocol switch button
 *
 * 按下协议切换键：在 insta360 与 DJI 之间切换协议，并用 LED 反馈。
 * 切到 DJI：黄灯双闪；切到 insta360：绿灯双闪。
 */
static void handle_proto_switch(void) {
    if (pairing_is_active()) {
        controller_pairing_exit();  /* 对频中切协议：先退出对频 */
    }

    backend_id_t cur = camera_backend_active_id();
    backend_id_t next = (cur == BACKEND_INSTA360) ? BACKEND_DJI : BACKEND_INSTA360;

    controller_switch_protocol(next);   /* 异步切换运行角色 + 持久化 */

    if (next == BACKEND_DJI) {
        light_logic_flash(13, 0, 0, 2);   /* 红灯双闪 = DJI */
    } else {
        light_logic_flash(0, 13, 0, 2);    /* 绿灯双闪 = insta360 */
    }
    ESP_LOGI(TAG, "Switched protocol to %s", (next == BACKEND_INSTA360) ? "insta360" : "DJI");
}

/**
 * @brief 处理协议切换键短按：切换「相机端」预设
 *        Handle short press of the second button: cycle the CAMERA-side preset
 *
 * 只上报一次 QS 键短按，切换到哪个预设由相机自身的快速切换列表决定
 * （见 docs/Q&A_CN.md 第 10 条）。控制器不保存、也不指定任何预设。
 * LED 用青色闪 2 次作为反馈。
 */
static void handle_camera_preset_next(void) {
    controller_preset_next();
    light_logic_flash(0, 13, 13, 2);   /* 青色双闪 = 已发送预设切换 */
    ESP_LOGI(TAG, "Camera preset -> next (QS key report)");
}

/**
 * @brief 处理双键同时长按事件
 *        Handle simultaneous long press of both buttons
 *
 * 同时长按两个按键 = 切换 WiFi SoftAP 开/关，并用 LED 反馈。
 */
static void handle_both_long_press() {
    web_server_wifi_toggle();
    bool on = web_server_wifi_is_on();
    ESP_LOGI(TAG, "Both long press: WiFi %s", on ? "ON" : "OFF");
    if (on) {
        light_logic_flash(0, 13, 0, 3);   /* 绿灯三闪 = WiFi 开 */
    } else {
        light_logic_flash(13, 0, 0, 3);   /* 红灯三闪 = WiFi 关 */
    }
}
/**
 * @brief 处理单击事件
 *        Handle single press event
 *
 * 单击 = 录制/快门切换，具体语义由活动后端实现（DJI：拍照/直播时开始录制、
 * 录制中停止；insta360：快门切换）。
 */
static void handle_boot_single_press() {
    if (pairing_is_active()) {
        return;  // 对频期间单击不触发拍录（协议尚未决定）
    }
    controller_single_press();
}

/**
 * @brief 按键扫描任务
 *        Key scan task
 * 
 * 定期检查按键状态，检测单击和长按事件，并触发相应的操作：
 * Periodically check key status, detect single press and long press events, and trigger corresponding operations:
 * - 长按：进行蓝牙断开、重连、相机协议连接等操作。
 * - Long press: perform Bluetooth disconnect, reconnect, camera protocol connection, etc.
 * - 单击：根据当前相机模式启动或停止录制。
 * - Single press: start or stop recording based on current camera mode, and switch camera mode.
 */
static void key_scan_task(void *arg) {
    while (1) {
        // 读取两个按键状态
        // Read both button states
        bool boot_down = (gpio_get_level(BOOT_KEY_GPIO) == 0);          // BOOT 按下=低电平
        bool proto_down = (gpio_get_level(PROTO_SWITCH_KEY_GPIO) == 1);  // 协议键按下=高电平

        if (boot_down && proto_down) {
            // 双键同时按下：优先处理，抑制单键逻辑
            // Both pressed: priority branch, suppress individual handling
            if (!s_both_pressed) {
                s_both_pressed = true;
                s_both_press_start = xTaskGetTickCount();
                s_both_triggered = false;
                // 复位单键状态，避免松开后误触发单击/长按
                // Reset single-key state to avoid false triggers on release
                key_pressed = false;
                s_proto_switch_pressed = false;
                current_key_event = KEY_EVENT_NONE;
            } else if (!s_both_triggered &&
                       (xTaskGetTickCount() - s_both_press_start) >= LONG_PRESS_THRESHOLD) {
                s_both_triggered = true;
                handle_both_long_press();
            }
        } else {
            s_both_pressed = false;

            // ---- BOOT 键：单击=拍录，长按=对频 ----
            if (boot_down && !key_pressed) { // 按键按下 / Key pressed
                key_pressed = true;
                key_press_start_time = xTaskGetTickCount();
                current_key_event = KEY_EVENT_NONE;
                // ESP_LOGI(TAG, "BOOT key pressed.");
            } else if (boot_down && key_pressed) { // 按键保持按下状态 / Key remains pressed
                TickType_t press_duration = xTaskGetTickCount() - key_press_start_time;

                if (press_duration >= LONG_PRESS_THRESHOLD && current_key_event != KEY_EVENT_LONG_PRESS) {
                    // 长按事件（持续按下达到阈值时立即触发）
                    // Long press event (triggered immediately when threshold is reached)
                    current_key_event = KEY_EVENT_LONG_PRESS;
                    // 处理长按事件：进入对频扫描
                    // Handle long press event: enter pairing scan
                    handle_boot_long_press();
                    // ESP_LOGI(TAG, "Long press detected. Duration: %lu ticks", press_duration);
                }
            } else if (!boot_down && key_pressed) { // 按键松开 / Key released
                key_pressed = false;
                TickType_t press_duration = xTaskGetTickCount() - key_press_start_time;

                if (press_duration < LONG_PRESS_THRESHOLD) {
                    // 单击事件 / Single press event
                    current_key_event = KEY_EVENT_SINGLE;
                    ESP_LOGI(TAG, "Single press detected. Duration: %lu ticks", press_duration);
                    // 处理单击事件：拍录控制 / Handle single press event: recording control
                    handle_boot_single_press();
                }

                // 可以不做额外操作，因为长按的触发已经在按下过程中处理了
                // No additional operation needed as long press is handled during the press
            }

            // ---- 协议切换按键：长按触发协议切换（按下=高电平） ----
            if (proto_down && !s_proto_switch_pressed) {
                s_proto_switch_pressed = true;
                s_proto_switch_press_start = xTaskGetTickCount();
                s_proto_switch_triggered = false;
            } else if (proto_down && s_proto_switch_pressed) {
                TickType_t dur = xTaskGetTickCount() - s_proto_switch_press_start;
                if (dur >= LONG_PRESS_THRESHOLD && !s_proto_switch_triggered) {
                    s_proto_switch_triggered = true;
                    handle_proto_switch();
                }
            } else if (!proto_down && s_proto_switch_pressed) {
                /* 松开：短按 = 切换相机预设，长按已触发过则忽略 */
                s_proto_switch_pressed = false;
                if (!s_proto_switch_triggered) {
                    handle_camera_preset_next();
                }
            }
        }

        vTaskDelay(KEY_SCAN_INTERVAL);  // 每隔一段时间扫描一次按键状态 / Scan key state periodically
    }
}

/**
 * @brief 初始化按键逻辑
 *        Initialize key logic
 * 
 * 配置按键的 GPIO 引脚，并启动按键扫描任务。
 * Configure GPIO pin for key and start key scan task.
 */
void key_logic_init(void) {
    // 配置引脚为输入
    // Configure pin as input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_KEY_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&io_conf);

    // 配置协议切换按键（GPIO1）：下拉输入，按下接 VCC（高电平）
    // Configure protocol-switch button (GPIO1): pull-down, active-high
    gpio_config_t proto_io_conf = {
        .pin_bit_mask = (1ULL << PROTO_SWITCH_KEY_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&proto_io_conf);

    // 启动按键扫描任务
    // Start key scan task
    xTaskCreate(key_scan_task, "key_scan_task", 2048, NULL, 2, NULL);
}

