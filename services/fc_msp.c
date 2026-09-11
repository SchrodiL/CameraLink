/* SPDX-License-Identifier: MIT */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_util.h"

#include "fc_msp.h"
#include "hardware_config.h"
#include "channel_map.h"
#include "gps_fusion.h"

#include "msp_uart.h"
#include "msp_protocol.h"
#include "msp_messages.h"

#define TAG "FC_MSP"

/* NVS：GPS 方向开关 */
#define FC_MSP_NVS_NAMESPACE   "fcmsp"
#define FC_MSP_NVS_KEY_GPSDIR  "gps_dir"

/* MSPv2 原生帧；MSP2_SET_TEXT 是 MSPv2-only 命令 */
#define MSP_LINK_VERSION       MSP_V2_NATIVE

/* 任务轮询粒度（毫秒）。OSD 要求 2Hz 送达，50ms 粒度足够。 */
#define FC_MSP_POLL_MS         50

/* MSP_RC 请求的超时。飞控没应答时最坏等这么久——这个上限决定了任务一轮的
 * 最坏耗时，从而影响 OSD 文本的送达抖动。 */
#define FC_MSP_RC_TIMEOUT_MS   50

/* 读飞控 GPS 的超时（读方向才用） */
#define FC_MSP_GPS_TIMEOUT_MS  50

/* GPS 的轮询周期（毫秒）。比任务周期慢一倍：GPS 变化本来就慢，
 * 没必要每轮都问，也免得每轮再叠一个最长 50ms 的等待。 */
#define FC_MSP_GPS_PERIOD_MS   100

/* OSD 文本队列深度。osd 每 500ms 提交 4 条，本任务每 50ms 清空一次，
 * 8 个槽位远远够用；队列满时丢帧而不是阻塞调用者。 */
#define FC_MSP_TEXT_Q_LEN      8

typedef struct {
    uint8_t idx;
    char    text[OSD_CUSTOM_MSG_MAX_LEN + 1];
} osd_text_msg_t;

static msp_host_t   s_msp;
static QueueHandle_t s_text_q = NULL;

/* GPS 方向（二选一）。默认「读飞控 GPS」—— 最常见的配置是飞控自己接了 GPS。 */
static volatile fc_msp_gps_dir_t s_gps_dir = FC_MSP_GPS_READ;

/* ------------------------------------------------------------------ */
/* OSD 文本                                                            */
/* ------------------------------------------------------------------ */

void fc_msp_send_osd_text(uint8_t idx, const char *text)
{
    if (s_text_q == NULL || idx >= OSD_CUSTOM_MSG_COUNT) {
        return;
    }

    osd_text_msg_t m;
    m.idx = idx;
    snprintf(m.text, sizeof(m.text), "%s", (text != NULL) ? text : "");

    /* 非阻塞发送：调用方是 osd_task，一旦在这里阻塞，OSD 刷新和状态采样都会停摆。
     * 队列满就丢这一条，下一轮 OSD 刷新会重发，画面上看不出来。 */
    if (xQueueSend(s_text_q, &m, 0) != pdTRUE) {
        ESP_LOGW(TAG, "OSD text queue full, drop slot %u", (unsigned)idx);
    }
}

/* 把队列里的文本全部发出去。返回是否发过——只有发过才需要 flush 应答。 */
static bool fc_msp_flush_text(void)
{
    osd_text_msg_t m;
    bool sent = false;

    while (xQueueReceive(s_text_q, &m, 0) == pdTRUE) {
        uint8_t payload[OSD_CUSTOM_MSG_MAX_LEN + 2];
        uint16_t len = msp_build_custom_msg(payload, m.idx, m.text);

        /* 只发不等应答；飞控的应答由下面的 flush 丢弃 */
        int rc = msp_host_send(&s_msp, MSP2_SET_TEXT, payload, len);
        if (rc != MSP_HOST_OK) {
            ESP_LOGW(TAG, "SET_TEXT custom_osd[%u] failed (%d)", (unsigned)m.idx, rc);
        }
        sent = true;
    }

    return sent;
}

/* ------------------------------------------------------------------ */
/* RC 通道                                                             */
/* ------------------------------------------------------------------ */

static void fc_msp_poll_rc(void)
{
    msp_packet_t reply;

    /* 安全复位解析状态机，避免上一轮残留的半帧污染本次请求 */
    msp_decoder_init(&s_msp.dec);

    if (msp_host_request(&s_msp, MSP_RC, NULL, 0, &reply, FC_MSP_RC_TIMEOUT_MS) == MSP_HOST_OK) {
        uint16_t ch[16];
        if (msp_decode_rc(&reply, ch)) {
            channel_map_feed(ch);
        }
    }
}

/* ------------------------------------------------------------------ */
/* GPS：读飞控 或 喂飞控（二选一）                                        */
/* ------------------------------------------------------------------ */

/* READ 方向：轮询 MSP_RAW_GPS，把飞控的定位交给融合模块 */
static void fc_msp_poll_gps(void)
{
    msp_packet_t reply;
    msp_decoder_init(&s_msp.dec);

    if (msp_host_request(&s_msp, MSP_RAW_GPS, NULL, 0, &reply,
                         FC_MSP_GPS_TIMEOUT_MS) != MSP_HOST_OK) {
        return;   /* 飞控没回：保持上一份样本，让它按时间自然过期 */
    }

    msp_raw_gps_t g;
    if (!msp_decode_raw_gps(&reply, &g)) {
        return;
    }

    gps_fusion_feed_fc(g.fix_type, g.num_sat,
                       (double)g.lat * 1e-7,             /* 1e-7 度 -> 度 */
                       (double)g.lon * 1e-7,
                       (double)g.alt_m,                  /* 已是米（hMSL）*/
                       (double)g.speed_cms / 100.0,      /* cm/s -> m/s */
                       (double)g.course_degx10 / 10.0,   /* 0.1 度 -> 度 */
                       (double)g.pdop_x100 * 0.01);      /* 老固件没有 PDOP 时为 0 */
}

/* WRITE 方向：把融合结果喂给飞控。
 * 即使没有定位也要发（fix=0）—— 告诉飞控「现在没 GPS」，否则它会一直
 * 保留我们之前喂进去的旧位置。 */
static void fc_msp_push_gps(void)
{
    gps_fused_t f;
    gps_fusion_get(&f);

    uint8_t  fix = 0, sats = 0;
    int32_t  lat = 0, lon = 0;
    uint16_t alt = 0, spd = 0;

    if (f.valid) {
        fix  = 3;                     /* 融合结果只在 3D 定位下才 valid */
        sats = f.num_sat;
        lat  = (int32_t)llround(f.lat * 1e7);
        lon  = (int32_t)llround(f.lon * 1e7);

        double a = f.alt_m;           /* 载荷里的 alt 单位是「米」 */
        if (a < 0.0) a = 0.0;
        if (a > 65535.0) a = 65535.0;
        alt = (uint16_t)(a + 0.5);

        double s = f.speed_ms * 100.0;   /* m/s -> cm/s */
        if (s < 0.0) s = 0.0;
        if (s > 65535.0) s = 65535.0;
        spd = (uint16_t)(s + 0.5);
    }

    uint8_t payload[16];
    uint16_t len = msp_build_set_raw_gps(payload, fix, sats, lat, lon, alt, spd);
    if (msp_host_send(&s_msp, MSP_SET_RAW_GPS, payload, len) != MSP_HOST_OK) {
        ESP_LOGW(TAG, "SET_RAW_GPS failed");
    }
}

/* ---- GPS 方向开关（二选一，持久化）---- */

void fc_msp_set_gps_dir(fc_msp_gps_dir_t dir)
{
    dir = (dir == FC_MSP_GPS_WRITE) ? FC_MSP_GPS_WRITE : FC_MSP_GPS_READ;
    if (dir == s_gps_dir) {
        return;
    }
    s_gps_dir = dir;
    ESP_LOGI(TAG, "GPS 方向 -> %s",
             (dir == FC_MSP_GPS_WRITE) ? "向飞控输出" : "读飞控");

    if (nvs_ensure_ready() == ESP_OK) {
        nvs_handle_t h;
        if (nvs_open(FC_MSP_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
            esp_err_t err = nvs_set_u8(h, FC_MSP_NVS_KEY_GPSDIR, (uint8_t)dir);
            if (err == ESP_OK) {
                err = nvs_commit(h);
            }
            nvs_close(h);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "persist gps_dir failed: %s", esp_err_to_name(err));
            }
        }
    }
}

fc_msp_gps_dir_t fc_msp_get_gps_dir(void)
{
    return s_gps_dir;
}

static void load_gps_dir(void)
{
    if (nvs_ensure_ready() != ESP_OK) {
        return;
    }
    nvs_handle_t h;
    if (nvs_open(FC_MSP_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    uint8_t v = 0;
    if (nvs_get_u8(h, FC_MSP_NVS_KEY_GPSDIR, &v) == ESP_OK && v <= FC_MSP_GPS_WRITE) {
        s_gps_dir = (fc_msp_gps_dir_t)v;
        ESP_LOGI(TAG, "GPS 方向（NVS）: %s",
                 (v == FC_MSP_GPS_WRITE) ? "向飞控输出" : "读飞控");
    }
    nvs_close(h);
}

/* ------------------------------------------------------------------ */
/* 任务                                                                */
/* ------------------------------------------------------------------ */

static void fc_msp_task(void *arg)
{
    (void)arg;
    TickType_t last_gps = 0;

    for (;;) {
        /* 先发 OSD 文本：它时间敏感（画面滞后最直观）。
         * RC 轮询放在后面——即使飞控没应答白等 50ms，也不会顺延 OSD 的送达。 */
        if (fc_msp_flush_text()) {
            /* 只有真的发过才 flush，避免空转时反复清接收缓冲 */
            msp_uart_flush_rx();
        }

        fc_msp_poll_rc();

        /* GPS 单独限速：变化慢，没必要每轮都问；也免得每轮再叠一个最长
         * 50ms 的等待（飞控不在时会把本任务明显拖慢）。 */
        TickType_t now = xTaskGetTickCount();
        if (now - last_gps >= pdMS_TO_TICKS(FC_MSP_GPS_PERIOD_MS)) {
            last_gps = now;
            if (s_gps_dir == FC_MSP_GPS_WRITE) {
                fc_msp_push_gps();
            } else {
                fc_msp_poll_gps();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(FC_MSP_POLL_MS));
    }
}

int fc_msp_init(void)
{
    msp_uart_config_t cfg = {
        .uart_num  = MSP_UART_NUM,
        .tx_pin    = MSP_TX_PIN,
        .rx_pin    = MSP_RX_PIN,
        .baud_rate = MSP_BAUD_RATE,
    };

    esp_err_t err = msp_uart_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "msp_uart_init failed: %s", esp_err_to_name(err));
        return -1;
    }
    msp_uart_bind(&s_msp, MSP_LINK_VERSION);

    load_gps_dir();

    s_text_q = xQueueCreate(FC_MSP_TEXT_Q_LEN, sizeof(osd_text_msg_t));
    if (s_text_q == NULL) {
        ESP_LOGE(TAG, "failed to create OSD text queue");
        return -1;
    }

    if (xTaskCreate(fc_msp_task, "fc_msp", 4096, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create fc_msp task");
        return -1;
    }

    ESP_LOGI(TAG, "MSP link started (UART%u, %d baud, v%d)",
             (unsigned)MSP_UART_NUM, MSP_BAUD_RATE, MSP_LINK_VERSION);
    return 0;
}
