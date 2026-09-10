/* SPDX-License-Identifier: MIT */

#ifndef PROFILE_H
#define PROFILE_H

#include <stdint.h>
#include <stdbool.h>

#include "osd_config.h"
#include "channel_map.h"

/*
 * profile.h — 3 套预设配置文件。
 *
 * 每个 profile 是一份配置快照：OSD 槽位 + 低电量报警阈值 + 通道映射。
 * 「切换 profile」= 把该 profile 的快照应用到 live 配置（osd_config /
 * channel_map 的 NVS），并持久化 active 索引。WebUI 也可把当前 live 配置
 * 存进某个 profile（profile_save）。
 */

#define PROFILE_COUNT 3

typedef struct {
    osd_item_t        slots[OSD_SLOT_COUNT];
    uint8_t           batt_alarm;
    channel_binding_t bindings[CHAN_FUNC_COUNT];
} profile_t;

/* 开机读取 active 索引（live 配置已由 osd_config/channel_map 各自从 NVS 载入）。 */
void profile_init(void);

/* 当前激活的 profile 索引 0..2。 */
uint8_t profile_get_active(void);

/* 切换激活 profile：把快照应用到 live 并持久化 active 索引。 */
void profile_set_active(uint8_t idx);

/* 把 p 存为第 idx 个 profile（idx 0..2）。 */
void profile_save(uint8_t idx, const profile_t *p);

/* 读取第 idx 个 profile 快照；成功返回 true。 */
bool profile_load(uint8_t idx, profile_t *out);

/* 从当前 live 配置组装一份快照（供 WebUI 保存 profile 用）。 */
void profile_from_live(profile_t *out);

#endif /* PROFILE_H */
