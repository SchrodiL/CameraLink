/* SPDX-License-Identifier: MIT */

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_log.h"

#include "gps_fusion.h"
#include "gps.h"

#define TAG "GPS_FUSION"

#define DEG_PER_RAD_INV   (180.0 / 3.14159265358979323846)

/* 融合结果：融合任务写，其它任务（osd_task / dji_gps）读 */
static portMUX_TYPE s_fused_lock = portMUX_INITIALIZER_UNLOCKED;
static gps_fused_t  s_fused;

/* 飞控样本：fc_msp 的任务写，融合任务读 */
static portMUX_TYPE s_fc_lock = portMUX_INITIALIZER_UNLOCKED;
static gps_sample_t s_fc_sample;
static bool         s_fc_have = false;

/* 健康计数：只有融合任务碰，不需要锁 */
static gps_health_t s_health;

static gps_fused_ready_cb_t s_ready_cb = NULL;

/* 与 gps.c 用同一个时钟（tick 毫秒），两边的时间戳才能直接比 */
static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static const char *src_name(gps_src_t s)
{
    switch (s) {
    case GPS_SRC_LOCAL: return "本地";
    case GPS_SRC_FC:    return "飞控";
    case GPS_SRC_BOTH:  return "融合";
    default:            return "无";
    }
}

void gps_fusion_feed_fc(uint8_t fix_type, uint8_t num_sat,
                        double lat, double lon, double alt_m,
                        double speed_ms, double course_deg, double pdop)
{
    gps_sample_t s;
    memset(&s, 0, sizeof(s));

    /* fix_type 的门槛（>=3D）交给算法判定，这里只区分「有/没有」 */
    s.valid    = (fix_type != 0);
    s.fix_type = fix_type;
    s.num_sat  = num_sat;
    s.lat = lat;
    s.lon = lon;
    s.alt_m = alt_m;
    s.course_deg = course_deg;
    s.pdop = pdop;

    /* 飞控只给**地面**速度与航向，据此合成水平分量。
     * 垂直分量飞控没有，保持 0 —— 融合时 vel_d 也不会采信飞控侧。 */
    double rad = course_deg / DEG_PER_RAD_INV;
    s.vel_n = speed_ms * cos(rad);
    s.vel_e = speed_ms * sin(rad);
    s.vel_d = 0.0;

    /* 注意：MSP_RAW_GPS 不提供米制精度（没有 hAcc），h_acc_m 保持 0，
     * 由算法按 PDOP（或退化到星数）推断。 */

    s.sample_ms = now_ms();

    taskENTER_CRITICAL(&s_fc_lock);
    s_fc_sample = s;
    s_fc_have = true;
    taskEXIT_CRITICAL(&s_fc_lock);
}

void gps_fusion_get(gps_fused_t *out)
{
    if (out == NULL) {
        return;
    }
    taskENTER_CRITICAL(&s_fused_lock);
    *out = s_fused;
    taskEXIT_CRITICAL(&s_fused_lock);
}

void gps_fusion_set_ready_cb(gps_fused_ready_cb_t cb)
{
    s_ready_cb = cb;
}

/* GPS_Data_t -> 统一样本 */
static void sample_from_local(const GPS_Data_t *g, gps_sample_t *s)
{
    memset(s, 0, sizeof(*s));

    bool ok = (g->Status == 1);
    s->valid    = ok;
    s->fix_type = ok ? 3 : 0;   /* GPS_Data 只留了 has_fix 布尔（= fixType >= 3）*/
    s->num_sat  = g->Num_Satellites;
    s->lat = g->Latitude;
    s->lon = g->Longitude;
    s->alt_m = g->Altitude;
    s->vel_n = g->Velocity_North;
    s->vel_e = g->Velocity_East;
    s->vel_d = g->Velocity_Descend;
    s->course_deg = g->Course;
    s->h_acc_m = g->H_Acc_M;
    s->v_acc_m = g->V_Acc_M;
    s->s_acc_mps = g->S_Acc_Mps;
    s->pdop = g->PDOP;
    s->sample_ms = g->Sample_Ms;
}

static void gps_fusion_task(void *arg)
{
    (void)arg;

    gps_src_t last_src = GPS_SRC_NONE;
    bool last_valid = false;

    for (;;) {
        GPS_Data_t  local;
        gps_sample_t ls, fc;
        bool fc_have;

        /* 本地：加锁快照（rx_task_GPS 随时在改） */
        gps_logic_snapshot(&local);
        sample_from_local(&local, &ls);

        /* 飞控：fc_msp 的任务写、这里读 */
        taskENTER_CRITICAL(&s_fc_lock);
        fc = s_fc_sample;
        fc_have = s_fc_have;
        taskEXIT_CRITICAL(&s_fc_lock);

        gps_fused_t out;
        gps_fusion_compute(&ls, fc_have ? &fc : NULL, now_ms(), &out, &s_health);

        taskENTER_CRITICAL(&s_fused_lock);
        s_fused = out;
        taskEXIT_CRITICAL(&s_fused_lock);

        /* 来源变化或有/无定位翻转时打一行 —— GPS 问题靠猜太费劲 */
        if (out.source != last_src || out.valid != last_valid) {
            last_src = out.source;
            last_valid = out.valid;
            ESP_LOGI(TAG, "GPS 来源 -> %s (本地 fix=%u sats=%u acc=%.1fm | "
                          "飞控 fix=%u sats=%u pdop=%.1f | 健康 %u/%u)",
                     src_name(out.source),
                     (unsigned)ls.fix_type, (unsigned)ls.num_sat, ls.h_acc_m,
                     (unsigned)(fc_have ? fc.fix_type : 0),
                     (unsigned)(fc_have ? fc.num_sat : 0),
                     fc_have ? fc.pdop : 0.0,
                     (unsigned)s_health.local, (unsigned)s_health.fc);
        }

        /* 只有有效才触发推送：没定位时推给相机没有意义。
         * 放在融合任务上下文——里面会做 BLE，绝不能在 osd_task 里。 */
        if (out.valid && s_ready_cb != NULL) {
            s_ready_cb();
        }

        vTaskDelay(pdMS_TO_TICKS(GPS_FUSION_PERIOD_MS));
    }
}

int gps_fusion_init(void)
{
    if (xTaskCreate(gps_fusion_task, "gps_fusion", 4096, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create gps_fusion task");
        return -1;
    }
    ESP_LOGI(TAG, "GPS fusion task started (%d ms 周期)", GPS_FUSION_PERIOD_MS);
    return 0;
}
