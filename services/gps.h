/* SPDX-License-Identifier: MIT
 * Copyright (c) 2025 SZ DJI Technology Co., Ltd.
 * Source: dji-sdk/Osmo-GPS-Controller-Demo (MIT)
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

    // Accuracy (from UBX-NAV-PVT; 0 = unknown)
    // 定位精度（来自 UBX-NAV-PVT，米制 1σ；0 表示未知）
    // UBX 本身就带这些字段，比 NMEA 的 HDOP 更直接，用于双源融合时给本地源定权重。
    double H_Acc_M;           // Horizontal accuracy (m)
                              // 水平精度（米）
    double V_Acc_M;           // Vertical accuracy (m)
                              // 垂直精度（米）
    double S_Acc_Mps;         // Speed accuracy (m/s)
                              // 速度精度（米/秒）
    double PDOP;              // Position dilution of precision (unitless)
                              // 位置精度因子（无量纲）

    // 本帧的采样时刻（毫秒，xTaskGetTickCount 时钟）。
    // 融合模块用它判断本地源是否还在更新——GNSS 挂掉时旧值会一直留在结构体里，
    // 只有时间戳能识别出「数据已经不新鲜了」。
    uint32_t Sample_Ms;

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

/* 取一份**一致的**快照（加锁拷贝）。跨任务读取必须走这里 ——
 * rx_task_GPS 随时在改写内部结构体，拿裸指针读会撕裂。 */
bool gps_logic_snapshot(GPS_Data_t *out);

#endif