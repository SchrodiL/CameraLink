/*
 * msp_protocol.h — MSP message IDs.
 *
 * MSP is the de-facto standard flight-controller serial protocol; listed here
 * are the message IDs most commonly used by a ground station / companion host.
 * The codec itself is message-agnostic: any uint16_t command id works, so
 * add more defines below as needed.
 */
#pragma once

/* ------------------------------------------------------------------ */
/* Info / identity (out = FC -> host)                                  */
/* ------------------------------------------------------------------ */
#define MSP_API_VERSION              1
#define MSP_FC_VARIANT               2
#define MSP_FC_VERSION               3
#define MSP_BOARD_INFO               4
#define MSP_BUILD_INFO               5
#define MSP_NAME                     10

/* ------------------------------------------------------------------ */
/* Configuration (out)                                                 */
/* ------------------------------------------------------------------ */
#define MSP_BATTERY_CONFIG           32
#define MSP_FEATURE_CONFIG           36
#define MSP_CURRENT_METER_CONFIG     40
#define MSP_VOLTAGE_METER_CONFIG     56

/* ------------------------------------------------------------------ */
/* Telemetry (out)                                                     */
/* ------------------------------------------------------------------ */
#define MSP_STATUS                   101
#define MSP_RAW_IMU                  102
#define MSP_SERVO                    103
#define MSP_MOTOR                    104
#define MSP_RC                       105
#define MSP_RAW_GPS                  106
#define MSP_COMP_GPS                 107
#define MSP_ATTITUDE                 108
#define MSP_ALTITUDE                 109
#define MSP_ANALOG                   110
#define MSP_RC_TUNING                111
#define MSP_PID                      112
#define MSP_BOXNAMES                 116
#define MSP_PIDNAMES                 117
#define MSP_VOLTAGE_METERS           128
#define MSP_CURRENT_METERS           129
#define MSP_BATTERY_STATE            130
#define MSP_UID                      160
#define MSP_ATTITUDE_QUATERNION      167

/* ------------------------------------------------------------------ */
/* Commands / settings (in = host -> FC)                               */
/* ------------------------------------------------------------------ */
#define MSP_SET_RAW_RC               200   /* 16 x uint16, NO reply */
#define MSP_SET_RAW_GPS              201
#define MSP_SET_PID                  202
#define MSP_SET_RC_TUNING            204
#define MSP_ACC_CALIBRATION          205
#define MSP_MAG_CALIBRATION          206
#define MSP_RESET_CONF               208
#define MSP_SET_HEADING              211
#define MSP_SET_MOTOR                214   /* 8 x uint16, NO reply */
#define MSP_SET_MOTOR_3D_CONFIG      217
#define MSP_SET_MOTOR_CONFIG         222

/* ------------------------------------------------------------------ */
/* MSPv2 (MSP2) commands — 0x3000+, MSPv2 native frames only.          */
/* 自定义 OSD 文本：MSPv2 扩展消息，需飞控固件支持。                     */
/* ------------------------------------------------------------------ */
#define MSP2_SET_TEXT                0x3007  /* in:  set a named text field */

/* MSP2_SET_TEXT / MSP2_GET_TEXT 的文本类型 ID */
#define MSP2TEXT_CUSTOM_MSG_0        7       /* 自定义 OSD 消息 0..3 */

#define OSD_CUSTOM_MSG_COUNT         4       /* 4 条自定义 OSD 消息 */
#define OSD_CUSTOM_MSG_MAX_LEN       16      /* 每条最大字符数 (MAX_NAME_LENGTH) */
