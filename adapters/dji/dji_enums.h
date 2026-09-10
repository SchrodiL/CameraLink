/* SPDX-License-Identifier: MIT */

#ifndef DJI_ENUMS_H
#define DJI_ENUMS_H

/*
 * dji_enums.h — DJI 协议专用枚举。
 *
 * cmd_type_t 描述命令帧/应答帧的应答需求；push_mode_t / push_freq_t 描述相机
 * 状态订阅方式。这些仅 DJI 后端使用，与协议无关的相机域枚举见 shared/camera_enums.h。
 */

typedef enum {
    CMD_NO_RESPONSE = 0x00,      // 命令帧 - 发送后不需要应答
    CMD_RESPONSE_OR_NOT = 0x01,  // 命令帧 - 需要应答，没收到不报错
    CMD_WAIT_RESULT = 0x02,      // 命令帧 - 需要应答，没收到会报错

    ACK_NO_RESPONSE = 0x20,      // 应答帧 - 不需要应答
    ACK_RESPONSE_OR_NOT = 0x21,  // 应答帧 - 需要应答，没收到不报错
    ACK_WAIT_RESULT = 0x22       // 应答帧 - 需要应答，没收到会报错
} cmd_type_t;

typedef enum {
    PUSH_MODE_OFF = 0,                    // 关闭
    PUSH_MODE_SINGLE,                     // 单次
    PUSH_MODE_PERIODIC,                   // 周期
    PUSH_MODE_PERIODIC_WITH_STATE_CHANGE  // 周期 + 状态变化推送
} push_mode_t;

typedef enum {
    PUSH_FREQ_2HZ = 20                    // 2Hz
} push_freq_t;

#endif /* DJI_ENUMS_H */
