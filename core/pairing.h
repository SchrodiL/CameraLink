/* SPDX-License-Identifier: MIT */

#ifndef PAIRING_H
#define PAIRING_H

#include <stdbool.h>

#include "camera_backend.h"

/*
 * pairing.h — 对频（绑定）编排：进入对频、识别相机、运行时切换后端（不重启）。
 *
 * 对频策略按「当前选定后端」分开，具体检测由各后端实现（见 camera_backend_t
 * 的 pairing_gap_handler）：
 *  - insta360：跑起从机广播，等相机主动连入；
 *  - DJI：扫描 DJI 相机广播，检测到厂商数据即通知本模块。
 *
 * 识别到之后不能在 BLE 回调里直接切换（回调跑在 bluedroid 任务里，会重入），
 * 因此后端检测只通过 pairing_notify_detected() 置 pending 标记，由 controller
 * 工作队列调用 pairing_poll() 在任务上下文里完成切换；insta360 连接边沿检测
 * 同样在 pairing_poll 里做。
 */

int pairing_init(void);
void pairing_enter(void);
void pairing_exit(void);
bool pairing_is_active(void);

/* 后端在对频期间检测到相机后调用，通知本模块在任务上下文里应用切换。 */
void pairing_notify_detected(backend_id_t id);

/* 由 controller 工作队列周期调用：insta360 连接边沿检测 + 应用待定检测结果。 */
void pairing_poll(void);

/* 注册「对频前关闭 WiFi」回调（web_server 在 init 时注册，避免 BLE/WiFi 共存干扰）。 */
void pairing_set_wifi_off_cb(void (*cb)(void));

#endif /* PAIRING_H */
