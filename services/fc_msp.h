/* SPDX-License-Identifier: MIT */

#ifndef FC_MSP_H
#define FC_MSP_H

#include <stdint.h>
#include <stdbool.h>

/*
 * fc_msp.h —— 与飞控的 MSP 链路（UART1）。
 *
 * MSP 是**单一资源**：msp_host 的解析状态机不是线程安全的，多任务共用会打乱
 * 解码。所以 UART、宿主和轮询全部集中在这里、由**一个任务独占**，其它模块只通过
 * 下面这几个接口使用它：
 *
 *   osd.c  → fc_msp_send_osd_text()   把合成好的文本交出去（内部入队）
 *   RC 通道 ← 由本模块的任务轮询 MSP_RC 后喂给 channel_map
 *
 * 之所以从 osd.c 里拆出来：osd.c 原先兼任「合成 OSD 文本」和「独占 MSP 链路」
 * 两件事，职责混在一起，加新的 MSP 交互（GPS 读写）会越来越乱。
 * 拆开后：**共用部分**（UART、宿主、任务循环）都在本文件；
 *          **差异部分**（每种消息的处理）各自一个函数，互不干扰。
 */

/* 拉起 MSP UART + 轮询任务。需在 osd_logic_init() 之前调用（OSD 要靠它发文本）。 */
int fc_msp_init(void);

/* 提交一条 OSD 文本（0..3 号 custom message）。非阻塞：队列满就丢弃，
 * 下一轮 OSD 刷新会重发，不影响画面。 */
void fc_msp_send_osd_text(uint8_t idx, const char *text);

/*
 * 飞控 GPS 的方向。做成**一个二选一开关**而不是两个独立开关，因为飞控侧
 * gps_provider 只能选一种，两者物理上互斥：都开的话，我们喂进去的数据会被
 * 飞控从 MSP_RAW_GPS 回读回来，形成自反馈。
 *
 *   READ  —— 用 MSP_RAW_GPS 读飞控的，参与融合（飞控自己接了 GPS）
 *   WRITE —— 把融合结果用 MSP_SET_RAW_GPS 喂给飞控（飞控没接 GPS，
 *            对应飞控侧 gps_provider = MSP）
 */
typedef enum {
    FC_MSP_GPS_READ = 0,
    FC_MSP_GPS_WRITE,
} fc_msp_gps_dir_t;

/* 设置/读取 GPS 方向。设置会持久化到 NVS 并立即生效。 */
void fc_msp_set_gps_dir(fc_msp_gps_dir_t dir);
fc_msp_gps_dir_t fc_msp_get_gps_dir(void);

#endif /* FC_MSP_H */
