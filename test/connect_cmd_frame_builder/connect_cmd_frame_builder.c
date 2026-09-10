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
 * If you receive this source code without DJI's authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

// 模拟 ESP-IDF 的日志系统，用于兼容现有代码
// Mock ESP-IDF logging system for compatibility with existing code
#define ESP_LOGE(TAG, fmt, ...) printf("[ERROR] %s: " fmt "\n", TAG, ##__VA_ARGS__)
#define ESP_LOGI(TAG, fmt, ...) printf("[INFO] %s: " fmt "\n", TAG, ##__VA_ARGS__)
#define ESP_LOGW(TAG, fmt, ...) printf("[WARN] %s: " fmt "\n", TAG, ##__VA_ARGS__)

// 复用 Demo 中的数据结构定义
// Reuse data structure definitions from the demo
#include "../../adapters/dji/dji_protocol_structures.h"
#include "../../shared/crc/custom_crc16.h"
#include "../../shared/crc/custom_crc32.h"

// 复用协议解析器的常量定义（从 dji_protocol_parser.c 复制）
// Reuse protocol parser constants (copied from dji_protocol_parser.c)
#define PROTOCOL_SOF_LENGTH 1
#define PROTOCOL_VER_LEN_LENGTH 2
#define PROTOCOL_CMD_TYPE_LENGTH 1
#define PROTOCOL_ENC_LENGTH 1
#define PROTOCOL_RES_LENGTH 3
#define PROTOCOL_SEQ_LENGTH 2
#define PROTOCOL_CRC16_LENGTH 2
#define PROTOCOL_CMD_SET_LENGTH 1
#define PROTOCOL_CMD_ID_LENGTH 1
#define PROTOCOL_CRC32_LENGTH 4

#define PROTOCOL_HEADER_LENGTH ( \
    PROTOCOL_SOF_LENGTH +        \
    PROTOCOL_VER_LEN_LENGTH +    \
    PROTOCOL_CMD_TYPE_LENGTH +   \
    PROTOCOL_ENC_LENGTH +        \
    PROTOCOL_RES_LENGTH +        \
    PROTOCOL_SEQ_LENGTH +        \
    PROTOCOL_CRC16_LENGTH +      \
    PROTOCOL_CMD_SET_LENGTH +    \
    PROTOCOL_CMD_ID_LENGTH)

#define PROTOCOL_TAIL_LENGTH PROTOCOL_CRC32_LENGTH

// 复用枚举定义（从 enums_logic.h 复制必要部分）
// Reuse enum definitions (copy necessary parts from enums_logic.h)
typedef enum {
    CMD_NO_RESPONSE = 0x00,
    CMD_RESPONSE_OR_NOT = 0x01,
    CMD_WAIT_RESULT = 0x02,
    ACK_NO_RESPONSE = 0x20,
    ACK_RESPONSE_OR_NOT = 0x21,
    ACK_WAIT_RESULT = 0x22
} cmd_type_t;

// 静态变量用于生成序列号
// Static variable for sequence number generation
static uint16_t s_current_seq = 0;

/**
 * @brief 生成序列号（复用 command_logic.c 中的逻辑）
 *        Generate sequence number (reuse logic from command_logic.c)
 * 
 * @return uint16_t 序列号 / Sequence number
 */
uint16_t generate_seq(void) {
    return ++s_current_seq;
}

/**
 * @brief 为连接请求创建有效载荷数据（复用 connection_data_creator 的逻辑）
 *        Create payload data for connection request (reuse connection_data_creator logic)
 * 
 * @param structure 连接请求结构体
 *                  Connection request structure
 * @param data_length 输出数据长度
 *                    Output data length
 * @param cmd_type 命令类型
 *                 Command type
 * @return uint8_t* 指向创建数据的指针，失败时返回NULL
 *                  Pointer to created data, NULL on failure
 */
uint8_t* connection_data_creator_standalone(const void *structure, size_t *data_length, uint8_t cmd_type) {
    if (structure == NULL || data_length == NULL) {
        ESP_LOGE("CONNECTION_CREATOR", "NULL input detected");
        return NULL;
    }

    uint8_t *data = NULL;

    if ((cmd_type & 0x20) == 0) {
        // 命令帧 / Command frame
        const connection_request_command_frame_t *command_frame = (const connection_request_command_frame_t *)structure;

        *data_length = sizeof(connection_request_command_frame_t);

        ESP_LOGI("CONNECTION_CREATOR", "Data length calculated for connection_request_command_frame_t: %zu", *data_length);

        data = (uint8_t *)malloc(*data_length);
        if (data == NULL) {
            ESP_LOGE("CONNECTION_CREATOR", "Memory allocation failed (command frame)");
            return NULL;
        }

        memcpy(data, command_frame, *data_length);
    } else {
        // 应答帧 / Response frame
        const connection_request_response_frame_t *response_frame = (const connection_request_response_frame_t *)structure;

        *data_length = sizeof(connection_request_response_frame_t);

        ESP_LOGI("CONNECTION_CREATOR", "Data length calculated for connection_request_response_frame_t: %zu", *data_length);

        data = (uint8_t *)malloc(*data_length);
        if (data == NULL) {
            ESP_LOGE("CONNECTION_CREATOR", "Memory allocation failed (response frame)");
            return NULL;
        }

        memcpy(data, response_frame, *data_length);
    }

    return data;
}

/**
 * @brief 创建协议帧（复用并简化 protocol_create_frame 的逻辑）
 *        Create protocol frame (reuse and simplify protocol_create_frame logic)
 * 
 * @param cmd_set 命令集
 *                Command set
 * @param cmd_id 命令ID
 *               Command ID  
 * @param cmd_type 命令类型
 *                 Command type
 * @param structure 数据结构指针
 *                  Data structure pointer
 * @param seq 序列号
 *            Sequence number
 * @param frame_length_out 输出帧长度
 *                         Output frame length
 * @return uint8_t* 指向创建的帧缓冲区的指针，失败时返回 NULL
 *                  Pointer to created frame buffer, NULL on failure
 */
uint8_t *protocol_create_frame_standalone(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, 
                                        const void *structure, uint16_t seq, size_t *frame_length_out)
{
    size_t data_length = 0;
    uint8_t *payload_data = NULL;

    // 创建有效载荷数据（复用连接数据创建器的逻辑）
    // Create payload data (reuse connection data creator logic)
    payload_data = connection_data_creator_standalone(structure, &data_length, cmd_type);

    // 处理空有效载荷的情况
    // Handle empty payload case
    if (payload_data == NULL && data_length > 0) {
        ESP_LOGE("PROTOCOL_FRAME", "Failed to create payload data with non-zero length");
        return NULL;
    }

    // 计算总帧长度
    // Calculate total frame length
    *frame_length_out = PROTOCOL_HEADER_LENGTH + data_length + PROTOCOL_TAIL_LENGTH;

    // 为完整帧分配内存
    // Allocate memory for complete frame
    uint8_t *frame = (uint8_t *)malloc(*frame_length_out);
    if (frame == NULL) {
        ESP_LOGE("PROTOCOL_FRAME", "Memory allocation failed for protocol frame");
        free(payload_data);
        return NULL;
    }

    // 初始化帧内容
    // Initialize frame content
    memset(frame, 0, *frame_length_out);

    // 填充协议头部（复用 protocol_create_frame 的逻辑）
    // Fill protocol header (reuse protocol_create_frame logic)
    size_t offset = 0;
    frame[offset++] = 0xAA; // SOF 起始字节 / SOF start byte

    // Ver/Length 字段
    // Ver/Length field
    uint16_t version = 0; // 固定版本号 / Fixed version number
    uint16_t ver_length = (version << 10) | (*frame_length_out & 0x03FF);
    frame[offset++] = ver_length & 0xFF;        // Ver/Length 低字节 / Ver/Length low byte
    frame[offset++] = (ver_length >> 8) & 0xFF; // Ver/Length 高字节 / Ver/Length high byte

    // 填充命令类型
    // Fill command type
    frame[offset++] = cmd_type;

    // ENC（不加密，固定 0）
    // ENC (no encryption, fixed 0)
    frame[offset++] = 0x00;

    // RES（保留字节，固定 0）
    // RES (reserved bytes, fixed 0)
    frame[offset++] = 0x00;
    frame[offset++] = 0x00;
    frame[offset++] = 0x00;

    // 序列号
    // Sequence number
    frame[offset++] = seq & 0xFF;        // 序列号低字节 / Low byte of sequence number
    frame[offset++] = (seq >> 8) & 0xFF; // 序列号高字节 / High byte of sequence number

    // 计算并填充 CRC-16（覆盖从 SOF 到 SEQ）
    // Calculate and fill CRC-16 (covers from SOF to SEQ)
    uint16_t crc16 = calculate_crc16(frame, offset);
    frame[offset++] = crc16 & 0xFF;        // CRC-16 低字节 / CRC-16 low byte
    frame[offset++] = (crc16 >> 8) & 0xFF; // CRC-16 高字节 / CRC-16 high byte

    // 填充命令集和命令 ID
    // Fill command set and ID
    frame[offset++] = cmd_set;
    frame[offset++] = cmd_id;

    // 填充有效载荷数据
    // Fill payload data
    if (payload_data != NULL) {
        memcpy(&frame[offset], payload_data, data_length);
    }
    offset += data_length;

    // 计算并填充 CRC-32（覆盖从 SOF 到 DATA）
    // Calculate and fill CRC-32 (covers from SOF to DATA)
    uint32_t crc32 = calculate_crc32(frame, offset);
    frame[offset++] = crc32 & 0xFF;         // CRC-32 第 1 字节 / CRC-32 byte 1
    frame[offset++] = (crc32 >> 8) & 0xFF;  // CRC-32 第 2 字节 / CRC-32 byte 2
    frame[offset++] = (crc32 >> 16) & 0xFF; // CRC-32 第 3 字节 / CRC-32 byte 3
    frame[offset++] = (crc32 >> 24) & 0xFF; // CRC-32 第 4 字节 / CRC-32 byte 4

    // 释放有效载荷数据
    // Free payload data
    if (payload_data != NULL) {
        free(payload_data);
    }

    return frame;
}

/**
 * @brief 以十六进制格式打印帧数据
 *        Print frame data in hex format
 * 
 * @param frame 帧数据
 *              Frame data
 * @param length 帧长度
 *               Frame length
 * @param prefix 打印前缀
 *               Print prefix
 */
void print_frame_hex(const uint8_t *frame, size_t length, const char *prefix) {
    printf("%s", prefix);
    for (size_t i = 0; i < length; i++) {
        printf("%02X", frame[i]);
    }
    printf("\n");
}

/**
 * @brief 打印帧结构分解
 *        Print detailed frame breakdown
 * 
 * @param frame 帧数据
 *              Frame data
 * @param length 帧长度
 *               Frame length
 */
void print_frame_breakdown(const uint8_t *frame, size_t length) {
    printf("\n帧结构分解 / Frame Breakdown:\n");
    printf("  SOF: %02X\n", frame[0]);
    printf("  Ver/Length: %02X%02X (Length=%zu)\n", frame[2], frame[1], length);
    printf("  CmdType: %02X\n", frame[3]);
    printf("  ENC: %02X\n", frame[4]);
    printf("  RES: %02X%02X%02X\n", frame[5], frame[6], frame[7]);
    printf("  SEQ: %02X%02X\n", frame[8], frame[9]);
    printf("  CRC16: %02X%02X\n", frame[10], frame[11]);
    printf("  CmdSet: %02X\n", frame[12]);
    printf("  CmdID: %02X\n", frame[13]);
    printf("  Payload: ");
    for (size_t i = 14; i < length - 4; i++) {
        printf("%02X", frame[i]);
    }
    printf("\n");
    printf("  CRC32: %02X%02X%02X%02X\n", 
           frame[length-4], frame[length-3], 
           frame[length-2], frame[length-1]);
}

/**
 * @brief 生成并打印相机连接请求命令帧（复用 key_logic.c 中的参数）
 *        Generate and print camera connection request command frame (reuse parameters from key_logic.c)
 * 
 * @param device_id 设备ID
 *                  Device ID
 * @param mac_addr MAC地址字节
 *                 MAC address bytes
 * @param mac_addr_len MAC地址长度
 *                     MAC address length
 * @param fw_version 固件版本
 *                   Firmware version
 * @param verify_mode 验证模式
 *                    Verification mode
 * @param verify_data 验证数据
 *                    Verification data
 * @param seq 序列号
 *            Sequence number
 */
void generate_connection_command_frame(uint32_t device_id, const int8_t *mac_addr, uint8_t mac_addr_len, 
                                     uint32_t fw_version, uint8_t verify_mode, uint16_t verify_data, 
                                     uint16_t seq) {
    printf("=== Camera Connection Request Command Frame Generator ===\n");
    printf("=== 相机连接请求命令帧生成器 ===\n\n");

    // 构造连接请求命令帧（复用 connect_logic.c 中的逻辑）
    // Construct connection request command frame (reuse logic from connect_logic.c)
    connection_request_command_frame_t connection_request = {
        .device_id = device_id,
        .mac_addr_len = mac_addr_len,
        .fw_version = fw_version,
        .conidx = 0,  // 保留字段 / Reserved field
        .verify_mode = verify_mode,
        .verify_data = verify_data,
    };
    
    // 复制MAC地址
    // Copy MAC address
    memset(connection_request.mac_addr, 0, sizeof(connection_request.mac_addr));
    if (mac_addr != NULL && mac_addr_len > 0) {
        size_t copy_len = mac_addr_len < sizeof(connection_request.mac_addr) ? 
                         mac_addr_len : sizeof(connection_request.mac_addr);
        memcpy(connection_request.mac_addr, mac_addr, copy_len);
    }
    
    // 清空保留字段
    // Clear reserved fields
    memset(connection_request.reserved, 0, sizeof(connection_request.reserved));

    // 打印参数
    // Print parameters
    printf("参数 / Parameters:\n");
    printf("  设备ID / Device ID: 0x%08X\n", device_id);
    printf("  MAC地址 / MAC Address: ");
    for (int i = 0; i < mac_addr_len; i++) {
        printf("%02X", (uint8_t)mac_addr[i]);
        if (i < mac_addr_len - 1) printf(":");
    }
    printf("\n");
    printf("  MAC长度 / MAC Length: %d\n", mac_addr_len);
    printf("  固件版本 / Firmware Version: 0x%08X\n", fw_version);
    printf("  验证模式 / Verify Mode: %d\n", verify_mode);
    printf("  验证数据 / Verify Data: 0x%04X\n", verify_data);
    printf("  序列号 / Sequence: 0x%04X\n", seq);
    printf("\n");

    // 生成协议帧（复用现有的协议创建逻辑）
    // Generate protocol frame (reuse existing protocol creation logic)
    size_t frame_length = 0;
    uint8_t *frame = protocol_create_frame_standalone(0x00, 0x19, CMD_WAIT_RESULT, &connection_request, seq, &frame_length);
    
    if (frame == NULL) {
        printf("ERROR: Failed to generate protocol frame\n");
        return;
    }

    printf("生成的帧 / Generated Frame:\n");
    printf("  帧长度 / Frame Length: %zu bytes\n", frame_length);
    printf("  十六进制数据 / Hex Data: ");
    print_frame_hex(frame, frame_length, "");
    
    // 打印帧结构分解
    // Print frame breakdown
    print_frame_breakdown(frame, frame_length);

    // 释放分配的内存
    // Free allocated memory
    free(frame);
}

/**
 * @brief 独立测试的主函数
 *        Main function for standalone testing
 */
int main(void) {
    printf("DJI Camera Connection Request Frame Builder\n");
    printf("DJI 相机连接请求帧构建器\n");
    printf("==========================================\n\n");

    // 示例参数（与key_logic.c中使用的相同）
    // Example parameters (same as used in key_logic.c)
    uint32_t device_id = 0x12345678;
    uint8_t mac_addr_len = 6;
    int8_t mac_addr[6] = {0x38, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    uint32_t fw_version = 0x00;
    uint8_t verify_mode = 0;  // 首次配对 / First pairing
    uint16_t verify_data = 0;
    uint16_t seq = generate_seq();

    // 像原始代码一样生成随机verify_data
    // Generate random verify_data like in the original code
    srand((unsigned int)time(NULL));
    verify_data = (uint16_t)(rand() % 10000);

    // 生成并打印连接请求帧
    // Generate and print the connection request frame
    generate_connection_command_frame(device_id, mac_addr, mac_addr_len, 
                                    fw_version, verify_mode, verify_data, seq);

    printf("\n注意：此帧可用于测试相机连接。\n");
    printf("Note: This frame can be used for testing camera connection.\n");
    printf("复制上述十六进制数据，通过您的测试工具发送。\n");
    printf("Copy the hex data above and send it via your testing tool.\n");

    return 0;
} 