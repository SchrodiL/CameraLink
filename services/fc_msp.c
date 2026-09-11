/* SPDX-License-Identifier: MIT */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "fc_msp.h"
#include "hardware_config.h"
#include "channel_map.h"

#include "msp_uart.h"
#include "msp_protocol.h"
#include "msp_messages.h"

#define TAG "FC_MSP"

/* MSPv2 原生帧；MSP2_SET_TEXT 是 MSPv2-only 命令 */
#define MSP_LINK_VERSION       MSP_V2_NATIVE

/* 任务轮询粒度（毫秒）。OSD 要求 2Hz 送达，50ms 粒度足够。 */
#define FC_MSP_POLL_MS         50

/* MSP_RC 请求的超时。飞控没应答时最坏等这么久——这个上限决定了任务一轮的
 * 最坏耗时，从而影响 OSD 文本的送达抖动。 */
#define FC_MSP_RC_TIMEOUT_MS   50

/* OSD 文本队列深度。osd 每 500ms 提交 4 条，本任务每 50ms 清空一次，
 * 8 个槽位远远够用；队列满时丢帧而不是阻塞调用者。 */
#define FC_MSP_TEXT_Q_LEN      8

typedef struct {
    uint8_t idx;
    char    text[OSD_CUSTOM_MSG_MAX_LEN + 1];
} osd_text_msg_t;

static msp_host_t   s_msp;
static QueueHandle_t s_text_q = NULL;

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
/* 任务                                                                */
/* ------------------------------------------------------------------ */

static void fc_msp_task(void *arg)
{
    (void)arg;

    for (;;) {
        /* 先发 OSD 文本：它时间敏感（画面滞后最直观）。
         * RC 轮询放在后面——即使飞控没应答白等 50ms，也不会顺延 OSD 的送达。 */
        if (fc_msp_flush_text()) {
            /* 只有真的发过才 flush，避免空转时反复清接收缓冲 */
            msp_uart_flush_rx();
        }

        fc_msp_poll_rc();

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
