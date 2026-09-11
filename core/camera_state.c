/* SPDX-License-Identifier: MIT */

#include "camera_state.h"

#include <string.h>

#include "camera_backend.h"
#include "pairing.h"
#include "gps.h"
#include "gps_fusion.h"

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

    /* GPS：用**融合后**的结果（本地 GNSS + 飞控 MSP 双源）。
     * 只有一个来源有定位时也能正常显示。 */
    gps_fused_t g;
    gps_fusion_get(&g);

    s_state.gps_valid   = g.valid;
    s_state.satellites  = g.num_sat;
    s_state.speed_ms    = g.speed_ms;
    s_state.altitude_m  = g.alt_m;
    s_state.lat         = g.lat;
    s_state.lon         = g.lon;
    s_state.gps_source  = (uint8_t)g.source;

    /* 这两个原本只描述**本地模块**，现在放宽成「有任一来源可用」：
     *   gps_connected —— OSD 卫星项据此决定显示卫星数还是 NO GPS
     *   gps_found     —— LED 据此决定紫/绿（有定位）
     * 否则「只有飞控有定位」时 OSD 会显示 NO GPS、LED 也显示无定位，与实际不符。 */
    s_state.gps_connected = is_gps_connected() || g.valid;
    s_state.gps_found     = is_gps_found()     || g.valid;
}

const camera_state_t *camera_state_get(void)
{
    return &s_state;
}
