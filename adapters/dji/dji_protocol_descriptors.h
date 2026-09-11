/* SPDX-License-Identifier: MIT
 * Copyright (c) 2025 SZ DJI Technology Co., Ltd.
 * Source: dji-sdk/Osmo-GPS-Controller-Demo (MIT)
 */

#ifndef DJI_PROTOCOL_DESCRIPTORS_H
#define DJI_PROTOCOL_DESCRIPTORS_H

#include <stdint.h>

/* Structure support */
/* 结构体支持 */
typedef uint8_t* (*data_creator_func_t)(const void *structure, size_t *data_length, uint8_t cmd_type);
typedef int (*data_parser_func_t)(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

typedef struct {
    uint8_t cmd_set;              // Command set identifier (CmdSet)
                                  // 命令集标识符 (CmdSet)
    uint8_t cmd_id;               // Command identifier (CmdID)
                                  // 命令标识符 (CmdID)
    data_creator_func_t creator;  // Data creation function pointer
                                  // 数据创建函数指针
    data_parser_func_t parser;    // Data parsing function pointer
                                  // 数据解析函数指针
} data_descriptor_t;
extern const data_descriptor_t data_descriptors[];
extern const size_t DATA_DESCRIPTORS_COUNT;

uint8_t* camera_mode_switch_creator(const void *structure, size_t *data_length, uint8_t cmd_type);
int camera_mode_switch_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

int version_query_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

uint8_t* record_control_creator(const void *structure, size_t *data_length, uint8_t cmd_type);
int record_control_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

uint8_t* gps_data_creator(const void *structure, size_t *data_length, uint8_t cmd_type);
int gps_data_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

uint8_t* connection_data_creator(const void *structure, size_t *data_length, uint8_t cmd_type);
int connection_data_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

uint8_t* camera_status_subscription_creator(const void *structure, size_t *data_length, uint8_t cmd_type);
int camera_status_subscription_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

int camera_status_push_data_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

int new_camera_status_push_data_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

uint8_t* key_report_creator(const void *structure, size_t *data_length, uint8_t cmd_type);
int key_report_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

uint8_t* camera_power_mode_switch_creator(const void *structure, size_t *data_length, uint8_t cmd_type);
int camera_power_mode_switch_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type);

#endif