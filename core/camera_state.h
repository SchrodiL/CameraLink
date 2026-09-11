/* SPDX-License-Identifier: MIT */

/*
 * camera_state.h — 公用相机状态缓存（协议无关）。
 *
 * 把 DJI 与 insta360 两种协议下各自采集到的相机状态，归一化成一份统一的
 * 结构体，供 OSD / LED 等消费方读取，无需关心当前运行的是哪种协议。
 *
 * 采用「拉取」模型：camera_state_refresh() 一次性读取所有状态源
 * （camera_backend_active_id / connect_logic / status_logic / insta360_logic /
 *  gps_logic）并写入缓存；由单一任务周期调用（见 osd_logic.c）。
 * 缓存只有该任务一个写者，读取方通过 camera_state_get() 拿到只读指针，
 * 因此无需加锁。
 */

#ifndef CAMERA_STATE_H
#define CAMERA_STATE_H

#include <stdint.h>
#include <stdbool.h>

/* 当前运行的相机协议 */
typedef enum {
    CAM_PROTO_DJI = 0,
    CAM_PROTO_INSTA360 = 1,
} camera_protocol_t;

/* 归一化后的连接阶段（协议无关，供 LED 等消费方统一显示） */
typedef enum {
    CAM_CONN_DISCONNECTED = 0,  /* 未连接 */
    CAM_CONN_CONNECTING,        /* 连接中（搜索/握手/对频） */
    CAM_CONN_CONNECTED,         /* 已连接就绪 */
} camera_conn_phase_t;

/* 电量文字档位：相机只按挡位上报电量时（insta360），OSD 显示档位词而不是百分比。
 * 有精确百分比的情况（DJI）用 BATT_LABEL_NONE，照常显示数字。 */
typedef enum {
    BATT_LABEL_NONE = 0,   /* 无档位概念，显示百分比 */
    BATT_LABEL_FULL,       /* FULL   75~100% */
    BATT_LABEL_HIGH,       /* HIGH   50~74%  */
    BATT_LABEL_MEDIUM,     /* MEDIUM 25~49%  */
    BATT_LABEL_LOW,        /* LOW    0~24%   */
} battery_label_t;

/* 归一化后的相机状态（所有字段按统一语义填写） */
typedef struct {
    camera_protocol_t protocol;   /* 当前协议 */

    bool     connected;           /* 相机链路已建立（协议就绪，等价 conn_phase == CONNECTED） */
    camera_conn_phase_t conn_phase; /* 连接阶段（供 LED 区分未连接/连接中/已连接） */
    bool     recording;           /* 正在录制 */
    uint32_t rec_seconds;         /* 录制已持续秒数（未录制时为 0） */
    uint8_t  mode;                /* 相机模式（DJI 枚举值；insta360 时为 0） */

    bool     gps_found;           /* GPS 模块有数据（is_gps_found） */
    bool     gps_connected;       /* GPS 模块在发送 UBX 数据（连接已建立，与 fix 无关） */
    bool     gps_valid;           /* 定位有效（RMC+GGA 均有效） */
    uint8_t  satellites;          /* 卫星数 */
    double   speed_ms;            /* 地速 (m/s) */
    double   altitude_m;          /* 海拔 (m) */
    double   lat;                 /* 纬度 (度) */
    double   lon;                 /* 经度 (度) */

    uint8_t  battery_pct;         /* 电量百分比下界 0-100 */
    uint8_t  battery_hi;          /* 电量百分比上界；0 或等于下界 = 无区间（精确值） */
    battery_label_t battery_label;/* 电量文字档位；BATT_LABEL_NONE = 用百分比 */
    bool     charging;            /* 正在充电（insta360 由心跳包判定；DJI 暂不提供） */
    uint8_t  res;                 /* 录制分辨率（DJI video_resolution 枚举） */
    uint8_t  fps_idx;             /* 帧率（DJI fps_idx 枚举） */
    uint8_t  photo_ratio;         /* 拍照比例（0-4:3，1-16:9） */
    uint16_t record_time;         /* 1D02 record_time 原值（录像=秒，连拍=毫秒） */
    uint16_t real_time_countdown; /* 实时倒计时（秒，>0 表示倒计时进行中） */
    uint32_t photo_countdown_ms;  /* 拍照定时档位（毫秒，0=未开定时） */
    char     mode_param[21];      /* 新协议(1D06)模式参数字符串，如 "8K30"；空串表示无 */
    char     mode_name[21];       /* 新协议(1D06)模式名字符串，如 "PORTRAIT"；空串表示无 */
    char     name[17];            /* 相机名称 / product_id */
    uint32_t device_id;           /* DJI 设备 ID（0xFF55=Action 6, 0xFF44=Action 5 Pro,
                                     0xFF33=Action 4, 0xFF66=Osmo 360；未知/insta360 为 0） */
    uint32_t remain_capacity_mb;  /* SD 剩余容量 (MB) */
    uint32_t remain_time_s;       /* 剩余录像时间 (秒) */
    uint32_t remain_photos;       /* 剩余可拍张数（insta360 拍照模式；0 表示未知） */
} camera_state_t;

/* 从所有状态源重新采样，写入缓存。 */
void camera_state_refresh(void);

/* 返回缓存只读指针（在 camera_state_refresh 后调用）。 */
const camera_state_t *camera_state_get(void);

#endif /* CAMERA_STATE_H */
