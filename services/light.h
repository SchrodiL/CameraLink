/* SPDX-License-Identifier: MIT
 * Copyright (c) 2025 SZ DJI Technology Co., Ltd.
 * Source: dji-sdk/Osmo-GPS-Controller-Demo (MIT)
 */

#ifndef LIGHT_H
#define LIGHT_H

#include <stdint.h>
#include <stdbool.h>

int init_light_logic(void);

/* 一次性反馈闪烁：按给定颜色闪 times 次（非阻塞）。供协议切换等事件反馈用。 */
void light_logic_flash(uint8_t red, uint8_t green, uint8_t blue, int times);

/* WiFi 配置模式：彩色流水灯（彩虹循环），替代常规状态灯；on=false 恢复状态灯。 */
void light_logic_set_rainbow(bool on);

#endif