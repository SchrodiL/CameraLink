/* SPDX-License-Identifier: MIT */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera_backend.h"

/*
 * controller.h — 统一控制门面 + 单一控制工作队列。
 *
 * 所有「阻塞型 / BLE 状态型」控制动作（拍录、快门、协议切换、对频）都通过
 * controller_* 投递到独立工作队列串行执行，避免在 osd_task / httpd / 按键扫描
 * 等上下文里直接调用阻塞的协议命令（原先 channel_map 在 10Hz OSD 任务里阻塞
 * 调用最长可达 5s，会把 OSD 刷新与 RC 读取一起卡死）。
 */

int controller_init(void);

/* 控制动作（fire-and-forget，投递到工作队列）。 */
void controller_single_press(void);
void controller_shutter(void);
void controller_record_start(void);
void controller_record_stop(void);
void controller_preset_next(void);

/* 电源类动作。wake_beacon 会阻塞到相机连上或超时（上限 15 秒），
 * 因此也走工作队列，不会卡住按键扫描 / OSD。
 * 不支持的后端会被静默跳过。 */
void controller_sleep_wake(void);
void controller_power_off(void);
void controller_wake_beacon(void);

void controller_switch_protocol(backend_id_t id);
void controller_pairing_enter(void);
void controller_pairing_exit(void);

#endif /* CONTROLLER_H */
