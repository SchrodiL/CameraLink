/* SPDX-License-Identifier: MIT */

#ifndef OSD_CONFIG_H
#define OSD_CONFIG_H

#include <stdint.h>

/*
 * osd_config.h — OSD 元素布局配置（内容池 + 槽位映射）。
 *
 * OSD 有 4 个自定义文本槽位（飞控 Custom Message 0..3），每个槽位显示哪种
 * 内容由本模块决定。内容池见 osd_item_t，用户（WebUI）可从池中任选 4 项，
 * 分别映射到 1~4 号槽位。配置持久化在 NVS，开机读取；set 后立即生效，
 * 无需重启（OSD 任务下一轮刷新即应用）。
 */

/* OSD 可显示的内容类型（内容池） */
typedef enum {
    OSD_ITEM_REC,         /* 录制状态 + 时长 */
    OSD_ITEM_GPS,             /* GPS 卫星数 */
    OSD_ITEM_SPEED_ALT,       /* 地速 + 海拔 */
    OSD_ITEM_BATTERY,         /* 相机电量百分比 */
    OSD_ITEM_SPEC,            /* 录制规格（分辨率/帧率） */
    OSD_ITEM_NAME_STORAGE,    /* 相机名称 + 剩余容量 */
    OSD_ITEM_COUNT
} osd_item_t;

/* OSD 槽位数量（固定 4 个 Custom Message） */
#define OSD_SLOT_COUNT 4

/* 读取当前槽位配置（开机从 NVS 读一次并缓存；未设置时用默认布局）。 */
void osd_config_get_slots(osd_item_t slots[OSD_SLOT_COUNT]);

/* 写入槽位配置到 NVS 并立即生效（WebUI 调用，无需重启）。 */
void osd_config_set_slots(const osd_item_t slots[OSD_SLOT_COUNT]);

/*
 * 低电量报警阈值（百分比 0..100），默认 25%；0 表示关闭报警。
 * 电量低于阈值时，OSD 电量元素的图标切换为电池符号并闪烁。
 * 接口留待 WebUI 调用；set 后立即生效，无需重启。
 */
uint8_t osd_config_get_batt_alarm(void);
void osd_config_set_batt_alarm(uint8_t pct);

#endif /* OSD_CONFIG_H */
