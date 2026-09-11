/* SPDX-License-Identifier: MIT */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * web_server.h — 内嵌 WebUI（WiFi SoftAP + HTTP 服务器）。
 *
 * 提供配置 / 状态 / 通道映射 / OTA / 备份的 REST 接口，并托管
 * 内嵌的 index.html（EMBED_FILES）。访问方式：手机/电脑连接 AP「CamLink」，
 * 浏览器打开 http://192.168.4.1/。
 */

/* 初始化 WiFi SoftAP + HTTP 服务器。返回 0 成功。 */
int web_server_init(void);

/* 切换 WiFi SoftAP 开/关（长按双键触发）。 */
void web_server_wifi_toggle(void);

/* 显式开/关 WiFi SoftAP。 */
void web_server_wifi_on(void);
void web_server_wifi_off(void);

/* WiFi SoftAP 当前是否开启。 */
bool web_server_wifi_is_on(void);

/* 自动开 WiFi 的延迟（秒）：上电后这么久无相机连接则自动开 WiFi。 */
uint32_t web_server_wifi_auto_delay_get(void);
void web_server_wifi_auto_delay_set(uint32_t sec);

#endif /* WEB_SERVER_H */
