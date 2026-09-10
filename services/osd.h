/* SPDX-License-Identifier: MIT */

/*
 * osd_logic.h — 通过 MSP 协议把相机状态写入飞控 OSD。
 *
 * 启动一个周期任务：采样公用相机状态缓存（camera_state），组合成 4 条
 * 自定义 OSD 文本，用 MSP2_SET_TEXT 通过 UART 发给飞控（Betaflight ≥ 4.6）。
 * 4 个「Custom Message」元素的位置/字体需在 Betaflight 地面站的 OSD 页里拖放。
 */

#ifndef OSD_H
#define OSD_H

/* 初始化 MSP UART 并启动 OSD 更新任务。返回 0 成功，-1 失败。 */
int osd_logic_init(void);

#endif /* OSD_H */
