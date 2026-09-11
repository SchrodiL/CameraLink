/* SPDX-License-Identifier: MIT
 * Copyright (c) 2025 SZ DJI Technology Co., Ltd.
 * Source: dji-sdk/Osmo-GPS-Controller-Demo (MIT)
 */

#ifndef DJI_BLE_H
#define DJI_BLE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "esp_err.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"

/* Connection status structure */
/* 连接状态结构体 */
typedef struct {
    bool is_connected; // Connection status
} connection_status_t;

/* Handle discovery status structure */
/* 特征句柄查找状态结构体 */
typedef struct {
    bool notify_char_handle_found; // Notify characteristic handle found
    bool write_char_handle_found;  // Write characteristic handle found
} handle_discovery_t;

/* Global profile structure to manage connection and characteristic information */
/* 为了简化，做一个全局的 profile 结构体来管理连接与特征信息 */
typedef struct {
    uint16_t conn_id;              // Connection ID
    esp_gatt_if_t gattc_if;        // GATT client interface

    /* Handles for characteristics we need to operate on */
    /* 根据需要记录我们要操作的特征 handle */
    uint16_t notify_char_handle;   // Notify characteristic handle
    uint16_t write_char_handle;    // Write characteristic handle
    uint16_t read_char_handle;     // Read characteristic handle

    /* Start and end handles of the service */
    /* service 的起始和结束 handle */
    uint16_t service_start_handle; // Service start handle
    uint16_t service_end_handle;   // Service end handle

    /* Remote device address */
    /* 远程设备地址 */
    esp_bd_addr_t remote_bda;      // Remote Bluetooth device address

    connection_status_t connection_status;     // Connection status
    handle_discovery_t handle_discovery;       // Handle discovery status
} ble_profile_t;

extern ble_profile_t s_ble_profile;

/**
 * @brief Notify callback function type for receiving data from remote
 * Notify 回调函数类型，用于接收从远端发来的数据
 *
 * @param data   Pointer to the notification data
 *               通知的数据指针
 * @param length Length of the notification data
 *               通知的数据长度
 */
typedef void (*ble_notify_callback_t)(const uint8_t *data, size_t length);

typedef void (*connect_logic_state_callback_t)(void);

esp_err_t ble_init();

esp_err_t ble_start_scanning_and_connect(void);

void ble_set_reconnecting(bool flag);

bool ble_get_reconnecting(void);

esp_err_t ble_reconnect(void);

esp_err_t ble_connect_to_address(esp_bd_addr_t addr);

esp_err_t ble_load_saved_peer(void);

bool ble_has_saved_peer(void);

esp_err_t ble_disconnect(void);

esp_err_t ble_write_without_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length);

esp_err_t ble_write_with_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length);

esp_err_t ble_register_notify(uint16_t conn_id, uint16_t char_handle);

void ble_set_notify_callback(ble_notify_callback_t cb);

void ble_set_state_callback(connect_logic_state_callback_t cb);

esp_err_t ble_start_advertising(void);

/* GAP event handler (forwarded by ble_common's unified GAP callback). */
void ble_client_gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/* True if the scan result is a DJI camera advertisement (manufacturer data AA 08 .. FA). */
bool ble_is_dji_camera_adv(esp_ble_gap_cb_param_t *scan_result);

/* Persist the remote peer address to NVS (used by pairing). */
esp_err_t ble_save_peer_addr(const esp_bd_addr_t addr);

/* 仅在 RAM 中记录对端地址（GAP/GATTC 回调可安全调用，不做 flash 写）。 */
void ble_note_peer_addr(const esp_bd_addr_t addr);

/* 把 RAM 中的对端地址落盘。只能在任务上下文调用，绝不可在 BT 回调里调用。 */
esp_err_t ble_persist_peer_addr(void);

/* Stop the DJI client role (disconnect + stop scan + unregister GATTC app + reset state). */
esp_err_t ble_client_stop(void);

#endif