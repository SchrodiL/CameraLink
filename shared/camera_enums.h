/* SPDX-License-Identifier: MIT */

#ifndef CAMERA_ENUMS_H
#define CAMERA_ENUMS_H

/*
 * camera_enums.h — 相机域枚举（协议无关）。
 *
 * 相机模式 / 状态 / 分辨率 / 帧率 / 防抖枚举，DJI 后端、insta360 后端以及
 * OSD 等消费方共用。协议专用枚举（cmd_type / push_mode / push_freq）见
 * adapters/dji/dji_enums.h。
 */

typedef enum {
    CAMERA_MODE_SLOW_MOTION          = 0x00,  // 慢动作 / Slow Motion
    CAMERA_MODE_NORMAL               = 0x01,  // 视频 / Video
    CAMERA_MODE_TIMELAPSE            = 0x02,  // 静止延时 / Timelapse
    CAMERA_MODE_PHOTO                = 0x05,  // 拍照 / Photo
    CAMERA_MODE_HYPERLAPSE           = 0x0A,  // 运动延时 / Hyperlapse
    CAMERA_MODE_LIVE_STREAMING       = 0x1A,  // 直播 / Live Streaming
    CAMERA_MODE_UVC_STREAMING        = 0x23,  // UVC 直播 / UVC Live Streaming
    CAMERA_MODE_SUPERNIGHT           = 0x28,  // 低光视频（超级夜景）/ SuperNight
    CAMERA_MODE_SUBJECT_TRACKING     = 0x34,  // 人物跟随 / Subject Tracking

    CAMERA_MODE_PANORAMIC_VIDEO_360  = 0x38, // 全景视频 / Panoramic Video (Osmo360)
    CAMERA_MODE_HYPERLAPSE_360       = 0x3A, // 运动延时 / Hyperlapse (Osmo360)
    CAMERA_MODE_SELFIE_360           = 0x3C, // 自拍模式 / Selfie Mode (Osmo360)
    CAMERA_MODE_PANORAMIC_PHOTO_360  = 0x3F, // 全景照片 / Panoramic Photo (Osmo360)
    CAMERA_MODE_BOOST_VIDEO_360      = 0x41, // 极广角视频 / Boost Video (Osmo360)
    CAMERA_MODE_VORTEX_360           = 0x43, // 旋转模式 / Vortex (Osmo360)
    CAMERA_MODE_PANORAMIC_SUPERNIGHT_360      = 0x44, // 全景超级夜景 / 360° SuperNight (Osmo360)
    CAMERA_MODE_SINGLE_LENS_SUPERNIGHT_360    = 0x4A  // 单镜头超级夜景 / Single Lens SuperNight (Osmo360)
} camera_mode_t;
const char* camera_mode_to_string(camera_mode_t mode);

typedef enum {
    CAMERA_STATUS_SCREEN_OFF = 0x00,          // 屏幕关闭 / Screen off
    CAMERA_STATUS_LIVE_STREAMING = 0x01,      // 直播 / Live streaming
    CAMERA_STATUS_PLAYBACK = 0x02,            // 回放 / Playback
    CAMERA_STATUS_PHOTO_OR_RECORDING = 0x03,  // 拍照或录像中 / Photo or recording
    CAMERA_STATUS_PRE_RECORDING = 0x05        // 预录制中 / Pre-recording
} camera_status_t;
const char* camera_status_to_string(camera_status_t status);

typedef enum {
    VIDEO_RESOLUTION_1080P = 10,         // 1920x1080P
    VIDEO_RESOLUTION_4K_16_9 = 16,       // 4096x2160P 4K 16:9
    VIDEO_RESOLUTION_2K_16_9 = 45,       // 2720x1530P 2.7K 16:9
    VIDEO_RESOLUTION_1080P_9_16 = 66,    // 1920x1080P 9:16
    VIDEO_RESOLUTION_2K_9_16 = 67,       // 2720x1530P 9:16
    VIDEO_RESOLUTION_2K_4_3 = 95,        // 2720x2040P 2.7K 4:3
    VIDEO_RESOLUTION_4K_4_3 = 103,       // 4096x3072P 4K 4:3
    VIDEO_RESOLUTION_4K_9_16 = 109,      // 4096x2160P 4K 9:16
    VIDEO_RESOLUTION_L = 4,              // 拍照画幅 L / Ultra Wide 30MP (Osmo360)
    VIDEO_RESOLUTION_M = 3,              // 拍照画幅 M / Wide 20MP (Osmo360)
    VIDEO_RESOLUTION_S = 2               // Standard 12MP (Osmo360)
} video_resolution_t;
const char* video_resolution_to_string(video_resolution_t res);

typedef enum {
    FPS_24 = 1,     // 24fps
    FPS_25 = 2,     // 25fps
    FPS_30 = 3,     // 30fps
    FPS_48 = 4,     // 48fps
    FPS_50 = 5,     // 50fps
    FPS_60 = 6,     // 60fps
    FPS_100 = 10,   // 100fps
    FPS_120 = 7,    // 120fps
    FPS_200 = 19,   // 200fps
    FPS_240 = 8     // 240fps
} fps_idx_t;
const char* fps_idx_to_string(fps_idx_t fps);

typedef enum {
    EIS_MODE_OFF = 0,      // 关闭 / Off
    EIS_MODE_RS = 1,       // RS
    EIS_MODE_RS_PLUS = 3,  // RS+
    EIS_MODE_HB = 4,       // HB
    EIS_MODE_HS = 2        // HS
} eis_mode_t;
const char* eis_mode_to_string(eis_mode_t mode);

#endif /* CAMERA_ENUMS_H */
