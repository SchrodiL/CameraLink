/* SPDX-License-Identifier: MIT
 * Copyright (c) 2025 SZ DJI Technology Co., Ltd.
 * Source: dji-sdk/Osmo-GPS-Controller-Demo (MIT)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "hardware_config.h"
#include "camera_state.h"

#define TAG "LOGIC_LIGHT"

// 创建一个 led_strip 句柄
// Create a led_strip handle
static led_strip_handle_t led_strip = NULL;

// 初始化 RGB LED 相关配置和设置
// Initialize RGB LED related configurations and settings
static void init_rgb_led(void) {
    // 配置 LED
    // Configure LED
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_STRIP_LENGTH // 设置 LED 数量为 1
                                     // Set the number of LEDs to 1
    };

    // 配置 RMT
    // Configure RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 设置 RMT 分辨率为 10 MHz
                                           // Set RMT resolution to 10 MHz
        .flags.with_dma = false,           // 禁用 DMA
                                           // Disable DMA
    };

    // 使用 &led_strip 作为第三个参数，因为它需要一个指向 led_strip_handle_t 的指针
    // Use &led_strip as the third parameter because it needs a pointer to led_strip_handle_t
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    led_strip_clear(led_strip); // 清除所有 LED，将其关闭
                                // Clear all LEDs and turn them off
    ESP_LOGI(TAG, "RGB LED initialized");
}

// 设置 RGB LED 的颜色
// Set RGB LED color
static void set_rgb_color(uint8_t red, uint8_t green, uint8_t blue) {
    led_strip_set_pixel(led_strip, 0, red, green, blue);  // 设置LED的颜色
                                                          // Set LED color
    led_strip_refresh(led_strip); // 刷新 LED Strip 以更新颜色
                                  // Refresh LED Strip to update color
}

// 初始化 RGB LED 状态所需的变量
// Initialize variables needed for RGB LED status
uint8_t led_red = 0, led_green = 0, led_blue = 0;   // RGB 值
                                                    // RGB values
bool led_blinking = false;                          // 是否闪烁
                                                    // Whether to blink
bool current_led_on = false;                        // LED 当前状态（开或关）
                                                    // Current LED status (on or off)

// 彩色流水灯（WiFi 配置模式）状态
// Rainbow running-light state (WiFi config mode)
static bool s_rainbow = false;
static uint16_t s_rainbow_hue = 0;   // 色相 0..360

static void update_led_state(void);

/* HSV -> RGB（h:0..360, s/v:0..255） */
static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (s == 0) { *r = *g = *b = v; return; }
    uint16_t region = h / 60;
    uint16_t rem = (h % 60) * 255 / 60;
    uint8_t p = (uint8_t)(v * (255 - s) / 255);
    uint8_t q = (uint8_t)(v * (255 - s * rem / 255) / 255);
    uint8_t t = (uint8_t)(v * (255 - s * (255 - rem) / 255) / 255);
    switch (region) {
    case 0: *r = v; *g = t; *b = p; break;
    case 1: *r = q; *g = v; *b = p; break;
    case 2: *r = p; *g = v; *b = t; break;
    case 3: *r = p; *g = q; *b = v; break;
    case 4: *r = t; *g = p; *b = v; break;
    default: *r = v; *g = p; *b = q; break;
    }
}

void light_logic_set_rainbow(bool on) {
    s_rainbow = on;
    if (!on) {
        update_led_state();   /* 恢复常规状态灯 */
    }
}

// 一次性反馈闪烁状态（协议切换等事件反馈，非阻塞）
// One-shot feedback flash state (e.g. protocol switch)
static bool s_feedback_active = false;
static int  s_feedback_toggles = 0;
static uint8_t s_feedback_r = 0, s_feedback_g = 0, s_feedback_b = 0;
static bool s_feedback_on = false;

/* 一次性反馈闪烁：按给定颜色闪 times 次。非阻塞，由 flash 定时器驱动。 */
void light_logic_flash(uint8_t red, uint8_t green, uint8_t blue, int times) {
    s_feedback_r = red;
    s_feedback_g = green;
    s_feedback_b = blue;
    s_feedback_toggles = times * 2;  /* 每次闪 = 亮 + 灭 */
    s_feedback_on = false;
    s_feedback_active = true;
}

// 更新 LED 状态的函数
// Function to update LED state
static void update_led_state() {
    /* 只读统一缓存；由 osd_task 周期调用 camera_state_refresh() 保持最新。 */
    const camera_state_t *st = camera_state_get();

    switch (st->conn_phase) {
    case CAM_CONN_DISCONNECTED:
        led_blinking = false;
        led_red = 13;      // 红 = 未连接
        led_green = 0;
        led_blue = 0;
        break;

    case CAM_CONN_CONNECTING:
        led_blinking = true;
        led_red = 0;
        led_green = 0;
        led_blue = 13;     // 蓝闪 = 连接中（搜索/握手/对频）
        break;

    case CAM_CONN_CONNECTED:
        if (st->recording) {
            led_blinking = true;
            if (st->gps_found) {
                led_red = 6;      // 紫闪 = 录制中且 GPS 已定位
                led_green = 0;
                led_blue = 6;
            } else {
                led_red = 0;
                led_green = 13;   // 绿闪 = 录制中但 GPS 未定位
                led_blue = 0;
            }
        } else {
            led_blinking = false;
            if (st->gps_found) {
                led_red = 6;      // 紫 = 待机且 GPS 已定位
                led_green = 0;
                led_blue = 6;
            } else {
                led_red = 0;
                led_green = 13;   // 绿 = 待机且 GPS 未定位
                led_blue = 0;
            }
        }
        break;

    default:
        led_red = 0;
        led_green = 0;
        led_blue = 0;
        break;
    }
}

// 定时器回调函数，用于定期更新 LED 状态
// Timer callback function for periodic LED state updates
static void led_state_timer_callback(TimerHandle_t xTimer) {
    if (s_feedback_active) {
        return;  // 反馈期间暂停状态刷新
    }
    update_led_state();
}

// 定时器回调函数，用于实现 LED 闪烁效果
// Timer callback function for LED blinking effect
static void led_blink_timer_callback(TimerHandle_t xTimer) {
    if (s_feedback_active) {
        return;  // 反馈期间暂停状态闪烁
    }
    if (s_rainbow) {
        return;  // 流水灯由独立的 rainbow 定时器驱动
    }
    if (led_blinking) {
        // 如果在闪烁状态，且当前 LED 开启，关闭 LED
        // If in blinking state and LED is currently on, turn it off
        if (current_led_on) {
            set_rgb_color(0, 0, 0);  // 关闭 LED
                                     // Turn off LED
        } else {
            // 如果当前 LED 关闭，设置为 RGB 颜色
            // If LED is currently off, set it to RGB color
            set_rgb_color(led_red, led_green, led_blue);
        }
        current_led_on = !current_led_on;
    } else {
        // 如果不在闪烁状态，直接设置为 RGB 颜色
        // If not in blinking state, directly set to RGB color
        set_rgb_color(led_red, led_green, led_blue);
    }
}

// 定时器回调函数，用于驱动一次性反馈闪烁
// Timer callback for one-shot feedback flash
static void led_flash_timer_callback(TimerHandle_t xTimer) {
    if (!s_feedback_active) {
        return;
    }
    s_feedback_on = !s_feedback_on;
    if (s_feedback_on) {
        set_rgb_color(s_feedback_r, s_feedback_g, s_feedback_b);
    } else {
        set_rgb_color(0, 0, 0);
    }
    s_feedback_toggles--;
    if (s_feedback_toggles <= 0) {
        s_feedback_active = false;
        update_led_state();  // 反馈结束，立即恢复状态显示
    }
}

/* 彩色流水灯：~33ms 步进（约 30 FPS），色相 +6（60 步 = 2 秒一个彩虹周期） */
static void led_rainbow_timer_callback(TimerHandle_t xTimer) {
    if (!s_rainbow || s_feedback_active) {
        return;
    }
    uint8_t r, g, b;
    hsv_to_rgb(s_rainbow_hue, 255, 64, &r, &g, &b);
    set_rgb_color(r, g, b);
    s_rainbow_hue = (s_rainbow_hue + 6) % 360;
}

// 初始化灯光逻辑，包括 LED 的状态更新和闪烁定时器
// Initialize light logic, including LED state updates and blink timer
int init_light_logic() {
    init_rgb_led();
    
    // 创建一个定时器，每 500ms 执行一次 update_led_state
    // Create a timer that executes update_led_state every 500ms
    TimerHandle_t led_state_timer = xTimerCreate("led_state_timer", pdMS_TO_TICKS(500), pdTRUE, (void *)0, led_state_timer_callback);

    if (led_state_timer != NULL) {
        xTimerStart(led_state_timer, 0);
        ESP_LOGI(TAG, "LED state timer started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create LED state timer");
        return -1;
    }

    // 创建另一个定时器，用来控制 LED 闪烁的状态（开关）
    // Create another timer to control LED blinking state (on/off)
    TimerHandle_t led_blink_timer = xTimerCreate("led_blink_timer", pdMS_TO_TICKS(500), pdTRUE, (void *)0, led_blink_timer_callback);

    if (led_blink_timer != NULL) {
        xTimerStart(led_blink_timer, 0);
        ESP_LOGI(TAG, "LED blink timer started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create LED blink timer");
        return -1;
    }

    // 创建反馈闪烁定时器（150ms，用于协议切换等一次性事件反馈）
    // Create flash timer (150ms) for one-shot event feedback
    TimerHandle_t led_flash_timer = xTimerCreate("led_flash_timer", pdMS_TO_TICKS(150), pdTRUE, (void *)0, led_flash_timer_callback);
    if (led_flash_timer != NULL) {
        xTimerStart(led_flash_timer, 0);
        ESP_LOGI(TAG, "LED flash timer started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create LED flash timer");
        return -1;
    }

    // 创建彩色流水灯定时器（~33ms 步进 ≈ 30 FPS，2 秒一个彩虹周期）
    // Create rainbow running-light timer (~33ms step, ~30 FPS, 2s per cycle)
    TimerHandle_t led_rainbow_timer = xTimerCreate("led_rainbow_timer", pdMS_TO_TICKS(33), pdTRUE, (void *)0, led_rainbow_timer_callback);
    if (led_rainbow_timer != NULL) {
        xTimerStart(led_rainbow_timer, 0);
        ESP_LOGI(TAG, "LED rainbow timer started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create LED rainbow timer");
        return -1;
    }
    return 0;
}
