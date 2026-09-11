/* SPDX-License-Identifier: MIT */

#include "dji_gps.h"

#include <stdlib.h>

#include "gps.h"
#include "gps_fusion.h"
#include "dji_connect.h"
#include "dji_command.h"
#include "dji_protocol_structures.h"

/* 把**融合后**的 GPS 结果打包成 0x0017 帧推给 DJI 相机。
 * 用融合结果而不是本地 GNSS 的原始值：只有一个来源有定位时也推得出去。 */
static void dji_gps_push(void)
{
    gps_fused_t f;
    gps_fusion_get(&f);
    if (!f.valid) {
        return;
    }

    /* 时间只有本地 GNSS 能提供（飞控的 MSP_RAW_GPS 不带时间字段）。
     * 只有飞控有定位时时间留 0，相机侧自己会忽略。 */
    GPS_Data_t local;
    gps_logic_snapshot(&local);
    int32_t year_month_day = (local.Year + 2000) * 10000 + local.Month * 100 + local.Day;
    int32_t hour_minute_second = (local.Hour + 8) * 10000 + local.Minute * 100 + (int32_t)local.Second;

    int32_t gps_longitude = (int32_t)(f.lon * 1e7);
    int32_t gps_latitude  = (int32_t)(f.lat * 1e7);
    int32_t height        = (int32_t)(f.alt_m * 1000);   /* mm */

    float speed_to_north   = f.vel_n * 100;   /* cm/s */
    float speed_to_east    = f.vel_e * 100;   /* cm/s */
    float speed_to_wnward  = f.vel_d * 100;   /* cm/s */

    /* 精度：用真实值，替掉之前写死的 1000/1000/10。
     * 本地源有 UBX 给的 hAcc/vAcc/sAcc；只有飞控时是按 PDOP×UERE 估的，
     * 仍比一个固定常量有意义。都拿不到时保守回落到 1m / 1m / 0.1m/s。 */
    uint32_t h_acc_mm  = (f.h_acc_m   > 0.0) ? (uint32_t)(f.h_acc_m   * 1000.0) : 1000u;
    uint32_t v_acc_mm  = (f.v_acc_m   > 0.0) ? (uint32_t)(f.v_acc_m   * 1000.0) : 1000u;
    uint32_t s_acc_cms = (f.s_acc_mps > 0.0) ? (uint32_t)(f.s_acc_mps * 100.0)  : 10u;

    gps_data_push_command_frame_t gps_frame = {
        .year_month_day = year_month_day,
        .hour_minute_second = hour_minute_second,
        .gps_longitude = gps_longitude,
        .gps_latitude = gps_latitude,
        .height = height,
        .speed_to_north = speed_to_north,
        .speed_to_east = speed_to_east,
        .speed_to_wnward = speed_to_wnward,
        .vertical_accuracy = v_acc_mm,
        .horizontal_accuracy = h_acc_mm,
        .speed_accuracy = s_acc_cms,
        .satellite_number = f.num_sat,
    };

    gps_data_push_response_frame_t *response = command_logic_push_gps_data(&gps_frame);
    if (response != NULL) {
        free(response);
    }
}

/* 融合结果就绪回调：仅在 DJI 已连接时推送。
 *
 * 挂在**融合模块**而不是本地 GNSS 的 ready 回调上 —— 原先挂在本地回调上时，
 * 「飞控有定位、本地没定位」这条路径永远不会触发，相机会一直收不到 GPS。
 * 本回调在融合任务上下文执行，里面的 BLE 发送不会拖累 OSD。 */
static void dji_gps_on_ready(void)
{
    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        return;
    }
    dji_gps_push();
}

void dji_gps_init(void)
{
    gps_fusion_set_ready_cb(dji_gps_on_ready);
}
