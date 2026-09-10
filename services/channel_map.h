/* SPDX-License-Identifier: MIT */

#ifndef CHANNEL_MAP_H
#define CHANNEL_MAP_H

#include <stdint.h>
#include <stdbool.h>

/*
 * channel_map.h — RC 通道到相机功能的映射（Betaflight Modes 风格）。
 *
 * 通过 MSP 从飞控读取 16 个 RC 通道值（1000..2000us），把每个「功能」绑定到
 * 一个 (通道, 值范围 [min,max])：通道值落在范围内即激活该功能。
 * 语义：
 *   - CHAN_FUNC_RECORD 是状态型：进入范围=开始录制，离开范围=停止录制。
 *   - 其余是边沿触发：进入范围触发一次。
 *
 * 映射关系与当前激活的 profile 联动：profile 切换会用 profile 内的快照覆盖
 * 这里的 live 绑定（见 profile.h）。
 */

typedef enum {
    CHAN_FUNC_NONE = 0,
    CHAN_FUNC_RECORD,         /* 状态型：范围内=录制，范围外=停止 */
    CHAN_FUNC_SHUTTER,        /* 边沿：快门/拍照 */
    CHAN_FUNC_PROTO_SWITCH,   /* 边沿：DJI <-> insta360 */
    CHAN_FUNC_PAIRING,        /* 边沿：触发对频 */
    CHAN_FUNC_PROFILE_1,      /* 边沿：切换到 profile 1 */
    CHAN_FUNC_PROFILE_2,      /* 边沿：切换到 profile 2 */
    CHAN_FUNC_PROFILE_3,      /* 边沿：切换到 profile 3 */
    CHAN_FUNC_COUNT
} channel_func_t;

typedef struct {
    uint8_t  channel;   /* RC 通道号 1..16；0 = 禁用 */
    uint16_t min;       /* 激活下限 (us)，通常 1000..2000 */
    uint16_t max;       /* 激活上限 (us) */
} channel_binding_t;

/* 开机从 NVS 读取绑定（无则默认全部禁用）。 */
void channel_map_init(void);

/* 读取当前 live 绑定。 */
void channel_map_get_bindings(channel_binding_t bindings[CHAN_FUNC_COUNT]);

/* 写入 live 绑定并持久化到 NVS（WebUI 调用）。 */
void channel_map_set_bindings(const channel_binding_t bindings[CHAN_FUNC_COUNT]);

/* 输入 16 通道值（ch[0]=通道1, ch[15]=通道16），执行映射与触发。 */
void channel_map_feed(const uint16_t ch[16]);

/* 返回最近一次读到的 16 通道值（供 WebUI 显示实时通道条）。 */
void channel_map_get_last_channels(uint16_t ch[16]);

#endif /* CHANNEL_MAP_H */
