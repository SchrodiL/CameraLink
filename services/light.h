/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) 2025 SZ DJI Technology Co., Ltd.
 *  
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
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