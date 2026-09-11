/* SPDX-License-Identifier: MIT */

#include "insta360_backend.h"

#include <string.h>

#include "camera_state.h"
#include "insta360.h"

static int insta360_init(void)
{
    return insta360_logic_init(false);
}

static void insta360_stop(void)
{
    insta360_logic_stop();
}

static bool be_insta360_is_connected(void)
{
    return insta360_is_connected();
}

static bool insta360_is_connecting(void)
{
    return false; /* insta360 无「连接中」中间态 */
}

static bool be_insta360_is_recording(void)
{
    return insta360_is_recording();
}

static void insta360_single_press(void)
{
    insta360_logic_shutter();
}

static void insta360_shutter(void)
{
    insta360_logic_shutter();
}

static void insta360_record_start(void)
{
    if (!insta360_is_recording()) {
        insta360_logic_shutter();
    }
}

static void insta360_record_stop(void)
{
    if (insta360_is_recording()) {
        insta360_logic_shutter();
    }
}

static void insta360_preset_next(void)
{
    insta360_logic_mode_next();
}

static void insta360_sleep_wake(void)
{
    insta360_logic_sleep_wake();
}

static void insta360_power_off(void)
{
    insta360_logic_power_off();
}

static void insta360_wake_beacon(void)
{
    insta360_logic_wake();
}

static void insta360_refresh_state(camera_state_t *st)
{
    st->recording = insta360_is_recording();
    st->rec_seconds = insta360_get_recording_seconds();
    st->mode = 0;
    st->battery_pct = insta360_get_battery_pct();
    st->battery_hi = insta360_get_battery_hi();
    st->battery_label = insta360_get_battery_label();
    st->charging = insta360_is_charging();
    st->res = 0;
    st->fps_idx = 0;
    st->photo_ratio = 0;
    st->record_time = 0;
    st->real_time_countdown = 0;
    st->photo_countdown_ms = 0;

    const char *mode = insta360_get_mode_str();
    if (mode != NULL && mode[0] != '\0') {
        strncpy(st->mode_param, mode, sizeof(st->mode_param) - 1);
        st->mode_param[sizeof(st->mode_param) - 1] = '\0';
    } else {
        st->mode_param[0] = '\0';
    }

    /* 拍摄模式名（SLOWMO/TIMELAPSE/…）。未确认的模式码留空，OSD 会回退显示规格串。 */
    const char *mode_name = insta360_get_mode_name();
    if (mode_name != NULL) {
        strncpy(st->mode_name, mode_name, sizeof(st->mode_name) - 1);
        st->mode_name[sizeof(st->mode_name) - 1] = '\0';
    } else {
        st->mode_name[0] = '\0';
    }

    const char *abbr = insta360_model_abbr(insta360_get_model());
    if (abbr != NULL) {
        strncpy(st->name, abbr, sizeof(st->name) - 1);
        st->name[sizeof(st->name) - 1] = '\0';
    } else {
        st->name[0] = '\0';
    }

    st->device_id = 0;
    st->remain_capacity_mb = 0;
    st->remain_time_s = insta360_get_remain_minutes() * 60;
    st->remain_photos = insta360_get_remain_photos();
}

const camera_backend_t insta360_backend = {
    .id = BACKEND_INSTA360,
    .name = "insta360",
    .init = insta360_init,
    .stop = insta360_stop,
    .is_connected = be_insta360_is_connected,
    .is_connecting = insta360_is_connecting,
    .is_recording = be_insta360_is_recording,
    .single_press = insta360_single_press,
    .shutter = insta360_shutter,
    .record_start = insta360_record_start,
    .record_stop = insta360_record_stop,
    .preset_next = insta360_preset_next,   /* 发通用「下一个模式」键，目标由相机决定 */
    .sleep_wake = insta360_sleep_wake,
    .power_off = insta360_power_off,
    .wake_beacon = insta360_wake_beacon,
    .refresh_state = insta360_refresh_state,
    .pairing_gap_handler = insta360_logic_gap_handler,
};

void insta360_backend_register(void)
{
    camera_backend_register(&insta360_backend);
}
