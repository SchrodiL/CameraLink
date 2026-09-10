/* SPDX-License-Identifier: MIT */

#ifndef CAMERA_BACKEND_H
#define CAMERA_BACKEND_H

#include <stdbool.h>
#include "esp_gap_ble_api.h"

#include "camera_state.h"

/*
 * camera_backend.h — 相机协议后端接口 + 活动后端注册/切换。
 *
 * 把「相机控制」抽象成协议无关的 vtable：DJI 与 insta360 各自实现一套
 * camera_backend_t 并注册。核心（key / channel_map / camera_state / osd /
 * web）只面对 camera_backend / controller，不感知具体协议——新增一种设备
 * 即新增一个 backend 实现。
 *
 * 本模块同时接管原先 protocol_config 的「活动协议 + NVS 持久化 + paired 标记
 * + 角色运行/停止」职责。
 */

typedef enum {
    BACKEND_DJI = 0,       // 大疆 Osmo Action / Osmo 360（GATTC 主机）
    BACKEND_INSTA360 = 1,  // insta360（GATTS 从机）
} backend_id_t;

typedef struct camera_backend {
    backend_id_t id;
    const char *name;

    /* 生命周期：启动/停止本后端角色（GATTC 主机 / GATTS 从机）。 */
    int  (*init)(void);
    void (*stop)(void);

    /* 连接状态。 */
    bool (*is_connected)(void);
    bool (*is_connecting)(void);
    bool (*is_recording)(void);

    /* 控制动作（fire-and-forget；由 controller 工作队列串行执行）。 */
    void (*single_press)(void);   /* BOOT 单击：录制/快门切换 */
    void (*shutter)(void);        /* 快门 / 拍照 */
    void (*record_start)(void);
    void (*record_stop)(void);

    /* 采样本后端状态，写入协议无关缓存（连接阶段由核心统一计算）。 */
    void (*refresh_state)(camera_state_t *st);

    /* 对频期间本后端的 GAP 事件处理（DJI：扫描检测相机；insta360：广播事件）。
     * 由 ble_common 在对频时按活动后端转发。 */
    void (*pairing_gap_handler)(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
} camera_backend_t;

/* ---- 注册表 ---- */
void camera_backend_register(const camera_backend_t *be);
const camera_backend_t *camera_backend_get(backend_id_t id);
const camera_backend_t *camera_backend_active(void);

/* ---- 活动协议（NVS 持久化） ---- */
backend_id_t camera_backend_active_id(void);
bool camera_backend_is_paired(void);
void camera_backend_set_paired(bool paired);

/* ---- 生命周期 ---- */
void camera_backend_select(backend_id_t id);       /* 仅持久化，不起角色、不标 paired */
void camera_backend_run_role(backend_id_t id);     /* 停当前角色、起目标角色 */
void camera_backend_stop_current(void);
void camera_backend_switch_to(backend_id_t id);    /* 跑角色 + 持久化 + 标记已配对 */

#endif /* CAMERA_BACKEND_H */
