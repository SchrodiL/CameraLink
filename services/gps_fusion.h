/* SPDX-License-Identifier: MIT */

#ifndef GPS_FUSION_H
#define GPS_FUSION_H

#include <stdint.h>
#include <stdbool.h>

#include "gps_fusion_math.h"

/*
 * gps_fusion.h —— GPS 双源融合的运行时。
 *
 * 两个来源：
 *   本地  —— 板载 u-blox LC76G（rx_task_GPS 解析 UBX-NAV-PVT）
 *   飞控  —— 经 MSP_RAW_GPS 读回（fc_msp 的任务轮询）
 *
 * 融合算法在 gps_fusion_math.c（纯算术、可主机单测）；本文件负责：
 *   - 一个周期任务：取本地快照 + 读飞控样本 → 算融合 → 发布结果
 *   - 跨任务安全：本地样本加锁快照，飞控样本加锁交换，融合结果加锁发布
 *   - 融合结果更新时触发回调（DJI 推送挂在这里）
 *
 * **为什么必须独立成任务**：DJI 推送要做 BLE，不能放在 osd_task 里（会拖慢 OSD）；
 * 而「只有飞控有定位」时本地没有 PVT 回调可触发推送，所以得有个周期源。
 */

/* 融合任务的采样周期（毫秒）。GPS 变化慢，10Hz 足够，也不占 CPU。 */
#define GPS_FUSION_PERIOD_MS   100

/* 拉起融合任务。 */
int gps_fusion_init(void);

/*
 * 喂入一份飞控样本（由 fc_msp 在解出 MSP_RAW_GPS 后调用，线程安全）。
 * 注意 speed/course 是飞控给的**地面**速度与航向，本模块会据此合成
 * vel_n/vel_e；飞控不提供垂直速度，vel_d 恒为 0。
 */
void gps_fusion_feed_fc(uint8_t fix_type, uint8_t num_sat,
                        double lat, double lon, double alt_m,
                        double speed_ms, double course_deg, double pdop);

/* 取融合结果快照（加锁拷贝）。 */
void gps_fusion_get(gps_fused_t *out);

/* 融合结果**有效**时每个周期回调一次（在融合任务上下文）。
 * DJI 后端的 GPS 推送挂在这里——原先它挂在本地 GNSS 的 ready 回调上，
 * 那样「只有飞控有定位」时永远推不出去。 */
typedef void (*gps_fused_ready_cb_t)(void);
void gps_fusion_set_ready_cb(gps_fused_ready_cb_t cb);

#endif /* GPS_FUSION_H */
