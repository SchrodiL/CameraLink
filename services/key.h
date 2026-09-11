/* SPDX-License-Identifier: MIT
 * Copyright (c) 2025 SZ DJI Technology Co., Ltd.
 * Source: dji-sdk/Osmo-GPS-Controller-Demo (MIT)
 */

#ifndef KEY_H
#define KEY_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "hardware_config.h"

// 按键事件
// Key Events
typedef enum {
    KEY_EVENT_NONE = 0,   // 无事件
                          // No event
    KEY_EVENT_SINGLE,     // 单击事件
                          // Single click event
    KEY_EVENT_LONG_PRESS, // 长按事件
                          // Long press event
    KEY_EVENT_ERROR       // 错误事件
                          // Error event
} key_event_t;

void key_logic_init(void);

#endif