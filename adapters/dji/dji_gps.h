/* SPDX-License-Identifier: MIT */

#ifndef DJI_GPS_H
#define DJI_GPS_H

/*
 * dji_gps.h — 把 GPS 服务解析出的 GPS_Data_t 打包成 DJI 0x0017 帧推送相机。
 *
 * GPS 解析在 services/gps（协议无关）；DJI 后端通过 gps_set_data_ready_cb 挂接
 * 本模块，在有有效定位且 DJI 已连接时推送。insta360 后端不消费 GPS 推送。
 */

void dji_gps_init(void);

#endif /* DJI_GPS_H */
