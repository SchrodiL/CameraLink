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

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"

#include "dji_ble.h"
#include "dji_data.h"
#include "camera_enums.h"
#include "dji_enums.h"
#include "dji_connect.h"
#include "dji_command.h"
#include "dji_status.h"
#include "dji_protocol_structures.h"

#define TAG "LOGIC_CONNECT"

/* 遥控器（ESP32）的 device ID，相机用它识别遥控器。
 * 0x12345678 为占位符；若 DJI 官方遥控器有固定 ID 可替换。 */
#define REMOTE_DEVICE_ID 0x12345678

static connect_state_t connect_state = BLE_NOT_INIT;

/* 上电自动重连任务（在 connect_logic_ble_init 中创建） */
static void connect_logic_auto_connect_task(void *arg);

/**
 * @brief Get current connection state
 *        获取当前连接状态
 * 
 * @return connect_state_t Returns current connection state
 *                        返回当前的连接状态
 */
connect_state_t connect_logic_get_state(void) {
    return connect_state;
}

/**
 * @brief Handle camera disconnection (callback function)
 *        处理相机断开连接（回调函数）
 * 
 * Perform operations according to current connection state and reset connection state to BLE initialization complete (BLE_INIT_COMPLETE).
 * 根据当前连接状态进行相应的操作，并将连接状态重置为 BLE 初始化完成（BLE_INIT_COMPLETE）。
 */
void receive_camera_disconnect_handler() {
    switch (connect_state) {
        case BLE_SEARCHING:
            break;
        case BLE_INIT_COMPLETE:
            ESP_LOGI(TAG, "Already in DISCONNECTED state.");
            break;
        case BLE_DISCONNECTING: {
            ESP_LOGI(TAG, "Normal disconnection process.");
            // Normal disconnection also needs to reset state
            // 正常断开也需要重置状态
            connect_state = BLE_INIT_COMPLETE;
            camera_status_initialized = false;
            ESP_LOGI(TAG, "Current state: DISCONNECTED.");
            break;
        }
        case BLE_CONNECTED:
        case PROTOCOL_CONNECTED:
        default: {
            ESP_LOGW(TAG, "Unexpected disconnection from state: %d, scheduling reconnection...", connect_state);

            /* 复位状态后，在独立 task 里重连。不能在 bluedroid 任务里阻塞调用
             * connect_logic_ble_connect()，否则 BLE 事件无法处理，连接永远无法完成，
             * 导致反复断联重连（原始项目用非阻塞 ble_reconnect() + 短等待，故正常）。 */
            connect_state = BLE_INIT_COMPLETE;
            camera_status_initialized = false;

            if (s_ble_profile.gattc_if != ESP_GATT_IF_NONE) {
                xTaskCreate(connect_logic_auto_connect_task, "auto_conn", 4096, NULL, 2, NULL);
                ESP_LOGI(TAG, "Reconnection task scheduled");
            } else {
                ESP_LOGW(TAG, "GATTC client not ready, skip reconnection");
            }
            break;
        }
    }
}

/**
 * @brief Initialize BLE connection
 *        初始化 BLE 连接
 * 
 * Initialize BLE and set state to BLE initialization complete (BLE_INIT_COMPLETE).
 * 初始化 BLE，并设置状态为 BLE 初始化完成（BLE_INIT_COMPLETE）。
 * 
 * @return int Returns 0 on success, -1 on failure
 *             成功返回 0，失败返回 -1
 */
int connect_logic_ble_init() {
    esp_err_t ret;

    /* 1. Initialize BLE (specify target device name to search and connect)
     * 1. 初始化 BLE（指定要搜索并连接的目标设备名） */
    ret = ble_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE, error: %s", esp_err_to_name(ret));
        return -1;
    }

    connect_state = BLE_INIT_COMPLETE;

    /* 载入上次连接设备的地址 */
    ble_load_saved_peer();

    /* 上电自动重连上次设备（若有保存的地址） */
    if (ble_has_saved_peer()) {
        xTaskCreate(connect_logic_auto_connect_task, "auto_conn", 4096, NULL, 2, NULL);
        ESP_LOGI(TAG, "Auto-connect task started");
    }

    ESP_LOGI(TAG, "BLE init successfully");
    return 0;
}

/**
 * @brief Stop the DJI connection role: stop client + reset state.
 * 停止 DJI 连接角色：停主机 + 复位状态。
 */
int connect_logic_ble_stop(void) {
    ble_client_stop();
    connect_state = BLE_INIT_COMPLETE;
    camera_status_initialized = false;
    ESP_LOGI(TAG, "BLE connect logic stopped");
    return 0;
}

/**
 * @brief Connect to BLE device
 *        连接到 BLE 设备
 * 
 * Execute the following steps: set callbacks, start scanning and attempt connection, wait for connection completion and characteristic handle discovery.
 * 执行以下步骤：设置回调、启动扫描并尝试连接、等待连接完成和特征句柄发现。
 * 
 * If connection fails, returns error and resets connection state.
 * 如果连接失败，会返回错误并重置连接状态。
 * 
 * @return int Returns 0 on success, -1 on failure
 *             成功返回 0，失败返回 -1
 */
/* 连接发起后的公共收尾流程：等待 BLE 链路建立、等待特征句柄发现、注册通知。 */
static int connect_logic_finish_connection(void) {
    /* 1. Wait up to 30 seconds to ensure BLE connection success */
    /* 等待最多 30 秒以确保 BLE 连接成功 */
    ESP_LOGI(TAG, "Waiting up to 30s for BLE to connect...");
    bool connected = false;
    for (int i = 0; i < 300; i++) { // 300 * 100ms = 30s
        if (s_ble_profile.connection_status.is_connected) {
            ESP_LOGI(TAG, "BLE connected successfully");
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!connected) {
        ESP_LOGW(TAG, "BLE connection timed out");
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 2. Wait for characteristic handle discovery completion (up to 30 seconds) */
    /* 等待特征句柄查找完成（最多等待30秒） */
    ESP_LOGI(TAG, "Waiting up to 30s for characteristic handles discovery...");
    bool handles_found = false;
    for (int i = 0; i < 300; i++) { // 300 * 100ms = 30s
        if (s_ble_profile.handle_discovery.notify_char_handle_found &&
            s_ble_profile.handle_discovery.write_char_handle_found) {
            ESP_LOGI(TAG, "Required characteristic handles found");
            handles_found = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!handles_found) {
        ESP_LOGW(TAG, "Characteristic handles not found within timeout");
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    /* 3. Register notification */
    /* 注册通知 */
    esp_err_t ret = ble_register_notify(s_ble_profile.conn_id, s_ble_profile.notify_char_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register notify, error: %s", esp_err_to_name(ret));
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    // Update state to BLE connected
    // 更新状态为 BLE 已连接
    connect_state = BLE_CONNECTED;

    // Delay RGB light display
    // 延迟展示氛围灯
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "BLE connect successfully");
    return 0;
}

int connect_logic_ble_connect(bool is_reconnecting) {
    connect_state = BLE_SEARCHING;

    esp_err_t ret;

    /* 1. Set a global Notify callback for receiving remote data and protocol parsing */
    /* 设置一个全局 Notify 回调，用于接收远端数据并进行协议解析 */
    ble_set_notify_callback(receive_camera_notify_handler);
    ble_set_state_callback(receive_camera_disconnect_handler);

    /* 2. Start scanning and attempt connection */
    /* 开始扫描并尝试连接 */
    ble_set_reconnecting(is_reconnecting);
    ret = ble_start_scanning_and_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scanning and connect, error: 0x%x", ret);
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    return connect_logic_finish_connection();
}

/* 直接连接指定地址（上次保存的设备），跳过扫描。 */
static int connect_logic_ble_connect_direct(void) {
    connect_state = BLE_SEARCHING;

    /* 设置全局 Notify 回调 */
    ble_set_notify_callback(receive_camera_notify_handler);
    ble_set_state_callback(receive_camera_disconnect_handler);

    esp_err_t ret = ble_connect_to_address(s_ble_profile.remote_bda);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start direct connect, error: 0x%x", ret);
        connect_state = BLE_INIT_COMPLETE;
        return -1;
    }

    return connect_logic_finish_connection();
}

/**
 * @brief Disconnect BLE connection
 *        断开 BLE 连接
 * 
 * Attempt to disconnect from BLE device.
 * 尝试断开与 BLE 设备的连接。
 * 
 * @return int Returns 0 on success, -1 on failure
 *             成功返回 0，失败返回 -1
 */
int connect_logic_ble_disconnect(void) {
    connect_state_t old_state = connect_state;
    connect_state = BLE_DISCONNECTING;
    
    ESP_LOGI(TAG, "Disconnecting camera...");

    // Call BLE layer's ble_disconnect function
    // 调用 BLE 层的 ble_disconnect 函数
    esp_err_t ret = ble_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect camera, BLE error: %s", esp_err_to_name(ret));
        connect_state = old_state;
        return -1;
    }

    ESP_LOGI(TAG, "Camera disconnected successfully");
    return 0;
}

/**
 * @brief Protocol connection function
 *        协议连接函数
 * 
 * This function is responsible for establishing protocol connection, including the following steps:
 * 该函数负责建立协议连接，包含以下步骤：
 * 
 * 1. Send connection request command to camera.
 *    向相机发送连接请求命令。
 * 2. Wait for camera's response and verify.
 *    等待相机的响应并进行验证。
 * 3. Send connection response according to camera's returned command.
 *    根据相机返回的命令发送连接应答。
 * 4. Set connection state to protocol connected.
 *    设置连接状态为协议连接。
 * 
 * @param device_id Device ID
 *                  设备ID
 * @param mac_addr_len MAC address length
 *                     MAC地址长度
 * @param mac_addr Pointer to MAC address
 *                 指向MAC地址的指针
 * @param fw_version Firmware version
 *                   固件版本
 * @param verify_mode Verification mode
 *                    验证模式
 * @param verify_data Verification data
 *                    验证数据
 * @param camera_reserved Camera reserved field
 *                        相机保留字段
 * 
 * @return int Returns 0 on success, -1 on failure
 *             成功返回 0，失败返回 -1
 */
int connect_logic_protocol_connect(uint32_t device_id, uint8_t mac_addr_len, const int8_t *mac_addr,
                                   uint32_t fw_version, uint8_t verify_mode, uint16_t verify_data,
                                   uint8_t camera_reserved) {
    ESP_LOGI(TAG, "%s: Starting protocol connection", __FUNCTION__);
    uint16_t seq = generate_seq();

    // Construct connection request command frame
    // 构造连接请求命令帧
    connection_request_command_frame_t connection_request = {
        .device_id = device_id,
        .mac_addr_len = mac_addr_len,
        .fw_version = fw_version,
        .verify_mode = verify_mode,
        .verify_data = verify_data,
    };
    memcpy(connection_request.mac_addr, mac_addr, mac_addr_len);


    // STEP1: Send connection request command to camera
    // 相机发送连接请求命令
    ESP_LOGI(TAG, "Sending connection request to camera...");
    CommandResult result = send_command(0x00, 0x19, CMD_WAIT_RESULT, &connection_request, seq, 1000);

    /**** Connection issue: camera may return either response frame or command frame ****/
    /****************** 连接问题，这里相机可能返回 应答帧 也可能返回 命令帧 ******************/

    if (result.structure == NULL) {
        // If a command frame is sent, execute this block of code
        // 如果发命令帧，走这里的代码

        // Directly call data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);
        // 这里直接去 esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);
        
        // If != OK, it means no message was received, timeout occurred
        // 如果 != OK 说明确实没有收到消息，超时
        
        // Otherwise, GOTO wait_for_camera_command label
        // 否则 GOTO 到 wait_for_camera_command 标识
        void *parse_result = NULL;
        size_t parse_result_length = 0;
        uint16_t received_seq = 0;
        esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 1000, &received_seq, &parse_result, &parse_result_length);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Timeout or error waiting for camera connection command, GOTO Failed.");
            connect_logic_ble_disconnect();
            return -1;
        } else {
            // If data is received, skip parsing camera response and directly enter STEP3
            // 如果能收到数据，跳过解析相机返回响应，直接进入STEP3
            goto wait_for_camera_command;
        }
    }

    // STEP2: Parse the response returned from camera
    // 解析相机返回的响应
    connection_request_response_frame_t *response = (connection_request_response_frame_t *)result.structure;
    if (response->ret_code != 0) {
        ESP_LOGE(TAG, "Connection handshake failed: unexpected response from camera, ret_code: %d", response->ret_code);
        free(response);
        connect_logic_ble_disconnect();
        return -1;
    }

    ESP_LOGI(TAG, "Handshake successful, waiting for the camera to actively send the connection command frame...");
    free(response);

    // STEP3: Wait for camera to send connection request
    // 等待相机主动发送连接请求
wait_for_camera_command:
    void *parse_result = NULL;
    size_t parse_result_length = 0;
    uint16_t received_seq = 0;
    esp_err_t ret = data_wait_for_result_by_cmd(0x00, 0x19, 30000, &received_seq, &parse_result, &parse_result_length);

    if (ret != ESP_OK || parse_result == NULL) {
        ESP_LOGE(TAG, "Timeout or error waiting for camera connection command");
        connect_logic_ble_disconnect();
        return -1;
    }

    // Parse the connection request command sent by camera
    // 解析相机发送的连接请求命令
    connection_request_command_frame_t *camera_request = (connection_request_command_frame_t *)parse_result;

    /* 保存相机型号 device_id（0xFF55=Action 6, 0xFF44=Action 5 Pro,
     * 0xFF33=Action 4, 0xFF66=Osmo 360），供 OSD 型号缩写使用 */
    current_camera_device_id = camera_request->device_id;
    ESP_LOGI(TAG, "Camera device_id: 0x%08X", (unsigned int)camera_request->device_id);

    if (camera_request->verify_mode != 2) {
        ESP_LOGE(TAG, "Unexpected verify_mode from camera: %d", camera_request->verify_mode);
        free(parse_result);
        connect_logic_ble_disconnect();
        return -1;
    }

    if (camera_request->verify_data == 0) {
        ESP_LOGI(TAG, "Camera approved the connection, sending response...");

        // Construct connection response frame
        // 构造连接应答帧
        connection_request_response_frame_t connection_response = {
            .device_id = device_id,
            .ret_code = 0,
        };
        memset(connection_response.reserved, 0, sizeof(connection_response.reserved));
        connection_response.reserved[0] = camera_reserved;

        ESP_LOGI(TAG, "Constructed connection response, sending...");

        // STEP4: Send connection response frame
        // 发送连接应答帧
        send_command(0x00, 0x19, ACK_NO_RESPONSE, &connection_response, received_seq, 5000);

        // Set connection state to protocol connected
        // 设置连接状态为协议连接
        connect_state = PROTOCOL_CONNECTED;

        ESP_LOGI(TAG, "Connection successfully established with camera.");
        free(parse_result);
        return 0;
    } else {
        ESP_LOGW(TAG, "Camera rejected the connection, closing Bluetooth link...");
        free(parse_result);
        connect_logic_ble_disconnect();
        return -1;
    }
}

/* 完整的相机连接流程：BLE 连接 → 协议连接 → 版本查询 → 订阅状态。
 * use_saved=true 且存在上次设备地址时直连，否则扫描信号最强的相机。 */
int connect_logic_connect_camera(bool use_saved) {
    /* 初始化数据层 */
    if (!is_data_layer_initialized()) {
        ESP_LOGI(TAG, "Data layer not initialized, initializing now...");
        data_init();
        data_register_status_update_callback(update_camera_state_handler);
        data_register_new_status_update_callback(update_new_camera_state_handler);
        if (!is_data_layer_initialized()) {
            ESP_LOGE(TAG, "Failed to initialize data layer");
            return -1;
        }
    }

    /* 若已连接，先断开 */
    connect_state_t current_state = connect_logic_get_state();
    if (current_state >= BLE_CONNECTED) {
        ESP_LOGI(TAG, "Current state is %d, disconnecting Bluetooth...", current_state);
        if (connect_logic_ble_disconnect() == -1) {
            ESP_LOGE(TAG, "Failed to disconnect Bluetooth.");
            return -1;
        }
    }

    /* BLE 连接：优先直连上次设备，失败回退扫描 */
    int res;
    if (use_saved && ble_has_saved_peer()) {
        ESP_LOGI(TAG, "Connecting to last device (direct)...");
        res = connect_logic_ble_connect_direct();
        if (res == -1) {
            ESP_LOGW(TAG, "Direct connect failed, falling back to scanning");
            res = connect_logic_ble_connect(false);
        }
    } else {
        ESP_LOGI(TAG, "Scanning for camera...");
        res = connect_logic_ble_connect(false);
    }
    if (res == -1) {
        ESP_LOGE(TAG, "Failed to connect Bluetooth.");
        return -1;
    }

    /* 相机协议连接：用 ESP32 真实 BLE MAC + 硬件随机验证码。 */
    uint32_t g_device_id = REMOTE_DEVICE_ID;
    uint8_t g_mac_addr_len = 6;
    int8_t g_mac_addr[6] = {0};
    esp_read_mac((uint8_t *)g_mac_addr, ESP_MAC_BT);
    uint32_t g_fw_version = 0x00;
    /* verify_mode=0：由相机根据已保存的配对历史决定是否弹窗验证（首次/重连都适用）。
     * 如需强制验证可改为 1（相机弹窗，用户确认）。 */
    uint8_t g_verify_mode = 0;
    uint16_t g_verify_data = (uint16_t)(esp_random() % 10000);
    uint8_t g_camera_reserved = 0;

    res = connect_logic_protocol_connect(g_device_id, g_mac_addr_len, g_mac_addr,
                                         g_fw_version, g_verify_mode, g_verify_data,
                                         g_camera_reserved);
    if (res == -1) {
        ESP_LOGE(TAG, "Failed to connect to camera.");
        return -1;
    }

    /* 获取设备版本信息，保存相机名称(product_id)供 OSD 显示 */
    version_query_response_frame_t *version_response = command_logic_get_version();
    if (version_response != NULL) {
        memcpy(current_product_id, version_response->product_id, 16);
        current_product_id[16] = '\0';
        free(version_response);
    }

    /* 订阅相机状态 */
    res = subscript_camera_status(PUSH_MODE_PERIODIC_WITH_STATE_CHANGE, PUSH_FREQ_2HZ);
    if (res == -1) {
        ESP_LOGE(TAG, "Failed to subscribe to camera status.");
        return -1;
    }

    ESP_LOGI(TAG, "Camera connected and status subscribed.");
    return 0;
}

/* 上电自动重连任务：等待 GATT 客户端注册完成后连接上次设备。 */
static void connect_logic_auto_connect_task(void *arg) {
    (void)arg;

    /* 等待 GATT 客户端注册完成（gattc_if 有效） */
    for (int i = 0; i < 100 && s_ble_profile.gattc_if == ESP_GATT_IF_NONE; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (s_ble_profile.gattc_if == ESP_GATT_IF_NONE) {
        ESP_LOGE(TAG, "GATT client not ready, abort auto-connect");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Auto-connecting to last connected device...");
    if (connect_logic_connect_camera(true) != 0) {
        ESP_LOGW(TAG, "Auto-connect failed");
    }
    vTaskDelete(NULL);
}

int connect_logic_ble_wakeup(void) {
    ESP_LOGI(TAG, "Attempting to wake up camera via BLE advertising");

    esp_err_t ret = ble_start_advertising();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %s", esp_err_to_name(ret));
        return -1;
    }

    ESP_LOGI(TAG, "BLE advertising started, attempting to wake up camera");
    return 0;
}
