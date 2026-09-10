/* SPDX-License-Identifier: MIT */

#ifndef DJI_BACKEND_H
#define DJI_BACKEND_H

#include "camera_backend.h"

/*
 * dji_backend.h — DJI 后端（GATTC 主机）的 camera_backend 实现。
 *
 * 把 dji_connect / dji_command / dji_status / dji_ble / dji_data 组装成统一的
 * camera_backend_t，供核心（camera_state / controller / pairing）调用。
 */

/* 注册 DJI 后端到 camera_backend 注册表，并挂接 GPS 数据推送钩子。 */
void dji_backend_register(void);

#endif /* DJI_BACKEND_H */
