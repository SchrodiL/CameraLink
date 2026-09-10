#ifndef INSTA360_H
#define INSTA360_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_gap_ble_api.h"

/*
 * insta360 remote logic (BLE GATT server role).
 * insta360 遥控器逻辑（BLE 从机角色）。
 *
 * The ESP32 impersonates the official "Insta360 GPS Remote" so the camera
 * connects to it. Scope: shutter + sleep/wake only.
 * ESP32 伪装成官方「Insta360 GPS Remote」，相机主动连入。范围：快门 + 休眠/唤醒。
 */

/*
 * 相机 BLE 设备名后 6 个 ASCII 字符，用于休眠唤醒广播（wake payload）。
 * 例：相机名 "X5 12AB34CD" -> 填 "AB34CD"。
 * 开机时会自动扫描相机广播名并提取后 6 位存入 NVS；学到之前用下面的默认值。
 */
#define INSTA360_DEFAULT_WAKE_PAYLOAD "123456"

/* Initialize the insta360 GATT server role + start advertising. Returns 0 on success.
 * BLE 栈已由 ble_stack_init 统一拉起，这里只初始化从机角色。
 * do_name_scan: 是否在唤醒 payload 仍为默认值时自动扫描相机名字学习（对频时不扫，避免与对频扫描冲突）。 */
int insta360_logic_init(bool do_name_scan);

/* Send shutter toggle command. */
void insta360_logic_shutter(void);

/* Send sleep (power off) command. */
void insta360_logic_sleep(void);

/* Send wake-up iBeacon advertisement (~3s). */
void insta360_logic_wake(void);

/* Set the wake payload (6 ASCII chars = camera BLE name suffix) and persist to NVS.
 * 供 app/WebUI 下发；开机自动学习也会写同一份 NVS。 */
void insta360_logic_set_wake_payload(const char last6[6]);

/* Whether a camera is currently connected. */
bool insta360_is_connected(void);

/* Whether the connected camera is recording (from timer packets). */
bool insta360_is_recording(void);

/* Elapsed recording seconds (0 when not recording). */
uint32_t insta360_get_recording_seconds(void);

/* Camera mode description string (e.g. "4K|30|DEW"), empty if unknown. */
const char *insta360_get_mode_str(void);

/* Remaining recording time in minutes (0 if unknown). */
uint32_t insta360_get_remain_minutes(void);

/* Remaining photo count (0 if unknown, photo mode). */
uint32_t insta360_get_remain_photos(void);

/* Camera battery percentage 0-100 (0 if unknown). */
uint8_t insta360_get_battery_pct(void);

/* Camera model name (e.g. "Ace Pro 2"), empty if unknown. */
const char *insta360_get_model(void);

/* Model abbreviation for OSD (e.g. "ACP2", "X5", "G3S"), NULL if unknown. */
const char *insta360_model_abbr(const char *model);

/* Start a one-shot camera name scan to learn model + wake payload (used during pairing). */
void insta360_logic_start_name_scan(void);

/* Stop the insta360 role (stop advertising + unregister GATTS app + reset state). */
void insta360_logic_stop(void);

/* GAP event handler (forwarded by ble_common's unified GAP callback). */
void insta360_logic_gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/* True if the BLE name is an insta360 camera (model prefix + 6-hex suffix). 对频识别用。 */
bool insta360_is_camera_name(const uint8_t *name, uint8_t len);

/* Extract the 6-hex wake payload (camera name suffix) into out[6]. */
void insta360_extract_wake_payload(const uint8_t *name, uint8_t len, char out[6]);

#endif
