/* SPDX-License-Identifier: MIT */

#include "camera_state.h"

#include <string.h>

#include "camera_backend.h"
#include "pairing.h"
#include "gps.h"

/* 节 (knots) -> 米/秒 的换算系数 */
#define KNOTS_TO_MPS 0.514444444

/* 唯一一份缓存。仅由 camera_state_refresh() 写入。 */
static camera_state_t s_state;

void camera_state_refresh(void)
{
    const camera_backend_t *be = camera_backend_active();

    /* 统一计算连接阶段（协议无关）：对频中视为「连接中」；其余按活动后端判定。 */
    if (pairing_is_active()) {
        s_state.conn_phase = CAM_CONN_CONNECTING;
    } else if (be != NULL && be->is_connected != NULL && be->is_connected()) {
        s_state.conn_phase = CAM_CONN_CONNECTED;
    } else if (be != NULL && be->is_connecting != NULL && be->is_connecting()) {
        s_state.conn_phase = CAM_CONN_CONNECTING;
    } else {
        s_state.conn_phase = CAM_CONN_DISCONNECTED;
    }
    s_state.connected = (s_state.conn_phase == CAM_CONN_CONNECTED);

    /* 协议相关字段由活动后端采样填充。 */
    if (be == NULL) {
        s_state.protocol = CAM_PROTO_DJI;
        s_state.recording = false;
        s_state.rec_seconds = 0;
        s_state.mode = 0;
        s_state.battery_pct = 0;
        s_state.res = 0;
        s_state.fps_idx = 0;
        s_state.photo_ratio = 0;
        s_state.record_time = 0;
        s_state.real_time_countdown = 0;
        s_state.photo_countdown_ms = 0;
        s_state.mode_param[0] = '\0';
        s_state.mode_name[0] = '\0';
        s_state.name[0] = '\0';
        s_state.device_id = 0;
        s_state.remain_capacity_mb = 0;
        s_state.remain_time_s = 0;
        s_state.remain_photos = 0;
    } else {
        s_state.protocol = (be->id == BACKEND_INSTA360) ? CAM_PROTO_INSTA360 : CAM_PROTO_DJI;
        if (be->refresh_state != NULL) {
            be->refresh_state(&s_state);
        }
    }

    /* GPS：两种协议共用同一份解析结果。 */
    s_state.gps_found = is_gps_found();
    s_state.gps_connected = is_gps_connected();
    const GPS_Data_t *gps = gps_logic_get_data();
    if (gps != NULL) {
        s_state.gps_valid = (gps->Status == 1);
        s_state.satellites = gps->Num_Satellites;
        s_state.speed_ms = gps->Speed_knots * KNOTS_TO_MPS;
        s_state.altitude_m = gps->Altitude;
        s_state.lat = gps->Latitude;
        s_state.lon = gps->Longitude;
    }
}

const camera_state_t *camera_state_get(void)
{
    return &s_state;
}
