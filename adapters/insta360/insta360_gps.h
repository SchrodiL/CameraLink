/* SPDX-License-Identifier: MIT */

#ifndef INSTA360_GPS_H
#define INSTA360_GPS_H

/* 初始化 insta360 GPS 上行（挂到 gps_fusion 的 ready 回调）。幂等。 */
void insta360_gps_init(void);

#endif /* INSTA360_GPS_H */
