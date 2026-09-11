/* SPDX-License-Identifier: MIT */

#ifndef BLE_COMMON_H
#define BLE_COMMON_H

/*
 * 统一 BLE 栈初始化 + 统一 GAP 回调。
 * 两个协议模块（DJI GATT 主机 / insta360 GATT 从机）共用同一条 BLE 栈：
 * 这里只做一次 controller/bluedroid 拉起，并把 GAP 事件按当前模式转发给对应模块。
 */

/* 初始化 BLE 栈（NVS + controller + bluedroid + 统一 GAP 回调）。成功返回 0。 */
int ble_stack_init(void);

#endif
