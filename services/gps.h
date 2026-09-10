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

#ifndef GPS_H
#define GPS_H

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "hardware_config.h"

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

// UART 配置（引脚/端口/波特率已集中到 hardware_config.h）
// UART config (pins/port/baud centralized in hardware_config.h)
#define RX_BUF_SIZE 800

typedef struct {
    // Time
    // 时间
    uint8_t Year;             // Year
                              // 年
    uint8_t Month;            // Month
                              // 月
    uint8_t Day;              // Day
                              // 日
    uint8_t Hour;             // Hour
                              // 时
    uint8_t Minute;           // Minute
                              // 分
    double Second;            // Second
                              // 秒

    // Position
    // 位置
    double Latitude;          // Latitude
                              // 纬度
    char Lat_Indicator;       // N/S
    double Longitude;         // Longitude
                              // 经度
    char Lon_Indicator;       // E/W

    // Other Information
    // 其他信息
    double Speed_knots;       // Ground Speed (knots)
                              // 地面速度 (节)
    double Course;            // Course (degrees)
                              // 航向 (度)
    double Altitude;          // Altitude (meters)
                              // 海拔高度 (米)
    uint8_t Num_Satellites;   // Number of Visible Satellites
                              // 可见卫星数量

    // Calculated Velocity Components
    // 计算后的速度分量
    double Velocity_North;    // Northward Velocity (m/s)
                              // 向北速度 (米/秒)
    double Velocity_East;     // Eastward Velocity (m/s)
                              // 向东速度 (米/秒)
    double Velocity_Descend;  // Descent Velocity (m/s)
                              // 下降速度 (米/秒)

    // Status
    // 状态
    uint8_t Status;          // 1: Both RMC and GGA valid, 0: Other cases
                             // 1: RMC和GGA都有效, 0: 其他情况
    uint8_t RMC_Valid;       // Whether RMC data is valid
                             // RMC 数据是否有效
    uint8_t GGA_Valid;       // Whether GGA data is valid
                             // GGA 数据是否有效
    double RMC_Latitude;     // Latitude from RMC
                             // RMC 的纬度
    double RMC_Longitude;    // Longitude from RMC
                             // RMC 的经度
    double GGA_Latitude;     // Latitude from GGA
                             // GGA 的纬度
    double GGA_Longitude;    // Longitude from GGA
                             // GGA 的经度
} GPS_Data_t;

void initSendGpsDataToCameraTask(void);

bool is_gps_found(void);

bool is_gps_connected(void);

bool is_current_gps_data_valid(void);

/* 返回最新解析的 GPS 数据（只读指针；由 rx_task_GPS 写入）。 */
const GPS_Data_t *gps_logic_get_data(void);

/* 数据就绪回调：收到有效定位数据时由 rx_task_GPS 调用。DJI 后端挂接此回调
 * 把 GPS 数据打包推送相机；insta360 后端不消费。 */
typedef void (*gps_data_ready_cb_t)(void);
void gps_set_data_ready_cb(gps_data_ready_cb_t cb);

#endif