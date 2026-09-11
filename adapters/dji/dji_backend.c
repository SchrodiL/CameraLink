/* SPDX-License-Identifier: MIT */

#include "dji_backend.h"

#include <string.h>
#include <stdlib.h>

#include "camera_state.h"
#include "camera_enums.h"
#include "dji_connect.h"
#include "dji_command.h"
#include "dji_status.h"
#include "dji_gps.h"
#include "dji_ble.h"
#include "pairing.h"

static const char *TAG = "DJI_BACKEND";

/* device_id -> DJI 规范型号名（相机发送的字节序不定，高低 16 位都试）。
 * 未知返回 NULL，由调用方回退到 product_id 原始字符串。 */
static const char *dji_model_name(uint32_t device_id)
{
    if (device_id == 0) return NULL;
    uint16_t lo = (uint16_t)device_id;
    uint16_t hi = (uint16_t)(device_id >> 16);
    if (lo == 0xFF55 || hi == 0xFF55) return "Action 6";
    if (lo == 0xFF44 || hi == 0xFF44) return "Action 5 Pro";
    if (lo == 0xFF33 || hi == 0xFF33) return "Action 4";
    if (lo == 0xFF66 || hi == 0xFF66) return "Osmo 360";
    return NULL;
}

static int dji_init(void)
{
    dji_gps_init(); /* 挂接 GPS 数据推送钩子（幂等）。 */
    return connect_logic_ble_init();
}

static void dji_stop(void)
{
    connect_logic_ble_stop();
}

static bool dji_is_connected(void)
{
    return connect_logic_get_state() == PROTOCOL_CONNECTED;
}

static bool dji_is_connecting(void)
{
    connect_state_t cs = connect_logic_get_state();
    return cs == BLE_SEARCHING || cs == BLE_CONNECTED;
}

static bool dji_is_recording(void)
{
    return is_camera_recording();
}

static void dji_single_press(void)
{
    camera_status_t status = (camera_status_t)current_camera_status;
    camera_mode_t mode = (camera_mode_t)current_camera_mode;

    if (mode == CAMERA_MODE_PHOTO || status == CAMERA_STATUS_LIVE_STREAMING) {
        record_control_response_frame_t *r = command_logic_start_record();
        if (r != NULL) {
            free(r);
        } else {
            connect_logic_ble_wakeup();
        }
    } else if (is_camera_recording()) {
        record_control_response_frame_t *r = command_logic_stop_record();
        if (r != NULL) {
            free(r);
        }
    }
}

static void dji_shutter(void)
{
    key_report_response_frame_t *r = command_logic_key_report_snapshot();
    if (r != NULL) {
        free(r);
    }
}

static void dji_record_start(void)
{
    if (!is_camera_recording()) {
        record_control_response_frame_t *r = command_logic_start_record();
        if (r != NULL) {
            free(r);
        }
    }
}

static void dji_record_stop(void)
{
    if (is_camera_recording()) {
        record_control_response_frame_t *r = command_logic_stop_record();
        if (r != NULL) {
            free(r);
        }
    }
}

/* 切换「相机端」预设：上报 QS 键短按。
 * 依官方文档（Q&A 第 10 条）：QS 键上报等价于短按相机上的 QS 键，
 * 目标模式由相机自身的快速切换列表决定，遥控器无需（也无法）指定，
 * 按一次进列表第一个模式，继续短按依次循环。 */
static void dji_preset_next(void)
{
    key_report_response_frame_t *r = command_logic_key_report_qs();
    if (r != NULL) {
        free(r);
    }
}

/* 睡眠/唤醒：读当前 power_mode 取反再下发。
 * 大疆这套是**绝对值设置**（0=正常 / 3=睡眠），而状态推送 1D02/1D06 会回报
 * 当前 power_mode，所以这里能确定性地切到目标状态 —— 不像 insta360 只能盲 toggle。
 * 相机未回报过状态时 current_camera_power_mode 为 0（正常），此时会切到睡眠，
 * 是合理的默认方向。 */
static void dji_sleep_wake(void)
{
    uint8_t target = (current_camera_power_mode == CAMERA_POWER_MODE_SLEEP)
                     ? CAMERA_POWER_MODE_NORMAL
                     : CAMERA_POWER_MODE_SLEEP;

    camera_power_mode_switch_response_frame_t *r = command_logic_set_power_mode(target);
    if (r != NULL) {
        free(r);
    }
}

/* 深度唤醒：相机已休眠/关机、链路不在时，靠**开始广播**把它叫回来。
 * 大疆协议里没有独立的「唤醒信标」，广播本机（伪装成官方遥控）就是唤醒手段。 */
static void dji_wake_beacon(void)
{
    connect_logic_ble_wakeup();
}

static void dji_refresh_state(camera_state_t *st)
{
    st->recording = is_camera_recording();
    st->rec_seconds = st->recording ? (uint32_t)current_record_time : 0;
    st->mode = current_camera_mode;
    st->battery_pct = current_camera_bat_percentage;
    st->battery_hi = current_camera_bat_percentage;  /* DJI 给的是精确值，无区间 */
    st->battery_label = BATT_LABEL_NONE;             /* DJI 显示精确百分比，不用档位词 */
    st->charging = false;   /* DJI 的状态推送里没有充电标志，显式清零避免残留 */
    st->res = current_video_resolution;
    st->fps_idx = current_fps_idx;
    st->photo_ratio = current_photo_ratio;
    st->record_time = current_record_time;
    st->real_time_countdown = current_real_time_countdown;
    st->photo_countdown_ms = current_photo_countdown_ms;

    strncpy(st->mode_param, (const char *)current_mode_param, sizeof(st->mode_param) - 1);
    st->mode_param[sizeof(st->mode_param) - 1] = '\0';
    strncpy(st->mode_name, (const char *)current_mode_name, sizeof(st->mode_name) - 1);
    st->mode_name[sizeof(st->mode_name) - 1] = '\0';

    const char *model = dji_model_name(current_camera_device_id);
    if (model != NULL) {
        strncpy(st->name, model, sizeof(st->name) - 1);
    } else {
        strncpy(st->name, current_product_id, sizeof(st->name) - 1);
    }
    st->name[sizeof(st->name) - 1] = '\0';

    st->device_id = current_camera_device_id;
    st->remain_capacity_mb = current_remain_capacity;
    st->remain_time_s = current_remain_time;
    st->remain_photos = 0;
}

/* 对频期间的 GAP 处理：扫描并检测 DJI 相机广播，检测到即通知 pairing。 */
static void dji_pairing_gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        ESP_LOGI(TAG, "Searching for DJI camera...");
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
            break;
        }
        if (ble_is_dji_camera_adv(param)) {
            esp_ble_gap_stop_scanning();
            /* 只记到 RAM：GAP 回调里做 NVS flash 写会阻塞 BT 栈。落盘在任务上下文完成。 */
            ble_note_peer_addr(param->scan_rst.bda);
            ESP_LOGI(TAG, "Detected DJI camera %02X:%02X:%02X:%02X:%02X:%02X",
                     param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2],
                     param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5]);
            pairing_notify_detected(BACKEND_DJI);
        }
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Pairing scan stopped");
        break;

    default:
        break;
    }
}

const camera_backend_t dji_backend = {
    .id = BACKEND_DJI,
    .name = "dji",
    .init = dji_init,
    .stop = dji_stop,
    .is_connected = dji_is_connected,
    .is_connecting = dji_is_connecting,
    .is_recording = dji_is_recording,
    .single_press = dji_single_press,
    .shutter = dji_shutter,
    .record_start = dji_record_start,
    .record_stop = dji_record_stop,
    .preset_next = dji_preset_next,
    .sleep_wake = dji_sleep_wake,
    /* power_off 留空：大疆协议没有「关机」命令（CmdSet/CmdID 全集里没有）。
     * 相机只能靠物理按键关闭；能远程控制的只有睡眠（0x00/0x1A）。 */
    .power_off = NULL,
    .wake_beacon = dji_wake_beacon,
    .refresh_state = dji_refresh_state,
    .pairing_gap_handler = dji_pairing_gap_handler,
};

void dji_backend_register(void)
{
    camera_backend_register(&dji_backend);
}
