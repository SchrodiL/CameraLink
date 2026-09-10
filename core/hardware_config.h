/* SPDX-License-Identifier: MIT */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

/*
 * hardware_config.h — 集中式硬件接线配置
 * Centralized hardware wiring configuration.
 *
 * 所有引脚、UART 端口、波特率集中在此文件。适配不同硬件布局时，
 * 只需修改这里，无需改动各逻辑模块。
 * All pin/UART/baud definitions live here. When adapting to a different
 * board layout, edit only this file.
 */

#include "driver/gpio.h"
#include "driver/uart.h"

/* ------------------------------------------------------------------ */
/* RGB LED（WS2812，RMT 驱动）                                          */
/* RGB LED (WS2812, RMT-driven)                                        */
/* ------------------------------------------------------------------ */
#define LED_GPIO            GPIO_NUM_8      /* LED 数据引脚 / data pin */
#define LED_STRIP_LENGTH    1               /* LED 数量 / number of LEDs */

/* ------------------------------------------------------------------ */
/* BOOT 按键                                                           */
/* BOOT button                                                         */
/* ------------------------------------------------------------------ */
#define BOOT_KEY_GPIO       GPIO_NUM_9      /* 按键引脚 / button pin */

/* ------------------------------------------------------------------ */
/* 协议切换按键（insta360 <-> DJI）                                     */
/* Protocol switch button (insta360 <-> DJI)                           */
/* ------------------------------------------------------------------ */
#define PROTO_SWITCH_KEY_GPIO  GPIO_NUM_1   /* 协议切换按键引脚 / protocol switch button pin */

/* ------------------------------------------------------------------ */
/* GPS（u-blox M8/M9/M10，HP UART0，UBX / NMEA 回退）                    */
/* GPS (u-blox M8/M9/M10, HP UART0, UBX / NMEA fallback)               */
/* 注：console 已切到 USB Serial/JTAG，释放 UART0 给 GPS                 */
/* ------------------------------------------------------------------ */
#define UART_GPS_PORT       UART_NUM_0
#define UART_GPS_TXD_PIN    GPIO_NUM_18
#define UART_GPS_RXD_PIN    GPIO_NUM_15
#define UART_GPS_BAUD_RATE  115200

/* ------------------------------------------------------------------ */
/* MSP / OSD（飞控遥测，HP UART）                                        */
/* MSP / OSD (flight-controller telemetry, HP UART)                    */
/* ------------------------------------------------------------------ */
#define MSP_UART_NUM        UART_NUM_1      /* ESP32-C6 HP UART1 */
#define MSP_TX_PIN          GPIO_NUM_20      /* ESP32-C6 TX -> 飞控 RX */
#define MSP_RX_PIN          GPIO_NUM_19      /* ESP32-C6 RX -> 飞控 TX */
#define MSP_BAUD_RATE       115200          /* Betaflight MSP 默认波特率 */

#endif /* HARDWARE_CONFIG_H */
