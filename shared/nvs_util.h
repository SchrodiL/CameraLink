/* SPDX-License-Identifier: MIT */

#ifndef NVS_UTIL_H
#define NVS_UTIL_H

#include "esp_err.h"

/*
 * nvs_util.h — NVS 初始化辅助。
 *
 * 各配置模块读写 NVS 前统一调用 nvs_ensure_ready()，取代此前散落在
 * protocol_config / osd_config / channel_map / profile / web_server 里
 * 各自复制的 ensure_nvs_ready() 样板。
 */

/* 初始化 NVS flash（幂等）。首次调用时若分区无空页或版本升级，自动擦除重试。 */
esp_err_t nvs_ensure_ready(void);

#endif /* NVS_UTIL_H */
