/* SPDX-License-Identifier: MIT */

#ifndef DJI_GPS_H
#define DJI_GPS_H

/*
 * dji_gps.h — 把**融合后**的 GPS 结果打包成 DJI 0x0017 帧推送相机。
 *
 * GPS 解析在 services/gps（协议无关），双源融合在 services/gps_fusion。
 * DJI 后端通过 gps_fusion_set_ready_cb 挂接本模块 —— 挂在**融合**上而不是
 * 本地 GNSS 的 ready 回调上，这样「只有飞控有定位」时也推得出去。
 */

void dji_gps_init(void);

#endif /* DJI_GPS_H */
