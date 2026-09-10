/* SPDX-License-Identifier: MIT */

#include "dji_gps.h"

#include <stdlib.h>

#include "gps.h"
#include "dji_connect.h"
#include "dji_command.h"
#include "dji_protocol_structures.h"

/* 把 GPS 服务解析出的 GPS_Data_t 打包成 0x0017 帧推送给 DJI 相机。 */
static void dji_gps_push(const GPS_Data_t *gps)
{
    int32_t year_month_day = (gps->Year + 2000) * 10000 + gps->Month * 100 + gps->Day;
    int32_t hour_minute_second = (gps->Hour + 8) * 10000 + gps->Minute * 100 + (int32_t)gps->Second;

    int32_t gps_longitude = (int32_t)(gps->Longitude * 1e7);
    int32_t gps_latitude = (int32_t)(gps->Latitude * 1e7);
    int32_t height = (int32_t)(gps->Altitude * 1000);   /* mm */

    float speed_to_north = gps->Velocity_North * 100;   /* cm/s */
    float speed_to_east = gps->Velocity_East * 100;     /* cm/s */
    float speed_to_wnward = gps->Velocity_Descend * 100;/* cm/s */
    uint32_t satellite_number = gps->Num_Satellites;

    gps_data_push_command_frame_t gps_frame = {
        .year_month_day = year_month_day,
        .hour_minute_second = hour_minute_second,
        .gps_longitude = gps_longitude,
        .gps_latitude = gps_latitude,
        .height = height,
        .speed_to_north = speed_to_north,
        .speed_to_east = speed_to_east,
        .speed_to_wnward = speed_to_wnward,
        .vertical_accuracy = 1000,    /* mm */
        .horizontal_accuracy = 1000,  /* mm */
        .speed_accuracy = 10,         /* cm/s */
        .satellite_number = satellite_number
    };

    gps_data_push_response_frame_t *response = command_logic_push_gps_data(&gps_frame);
    if (response != NULL) {
        free(response);
    }
}

/* GPS 服务数据就绪回调：仅在 DJI 已连接时推送。 */
static void dji_gps_on_ready(void)
{
    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        return;
    }
    const GPS_Data_t *gps = gps_logic_get_data();
    if (gps == NULL) {
        return;
    }
    dji_gps_push(gps);
}

void dji_gps_init(void)
{
    gps_set_data_ready_cb(dji_gps_on_ready);
}
