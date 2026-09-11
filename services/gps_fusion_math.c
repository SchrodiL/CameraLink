/* SPDX-License-Identifier: MIT */

#include "gps_fusion_math.h"

#include <math.h>
#include <string.h>

/* 健康滞回常量，照抄 ArduPilot AP_GPS_Blended：
 * 失败 +10（上限 100）、成功 −1；达到 50 判为不健康、停止参与融合。
 * 即约 5 次连续失败才停、50 次连续成功才完全恢复，避免在边界上反复进出。 */
#define HEALTH_FAIL_INC   10
#define HEALTH_STOP_AT    50

#define DEG_PER_RAD_INV   (180.0 / 3.14159265358979323846)

static void health_update(uint8_t *h, bool ok)
{
    if (!ok) {
        int v = (int)(*h) + HEALTH_FAIL_INC;
        *h = (v > 100) ? 100 : (uint8_t)v;
    } else if (*h > 0) {
        (*h)--;
    }
}

/* 样本本身是否可用（不含健康度判定） */
bool gps_sample_usable(const gps_sample_t *s, uint32_t now_ms)
{
    if (s == NULL || !s->valid) {
        return false;
    }
    if (s->fix_type < 3) {
        return false;
    }
    if (s->num_sat < GPS_FUSION_MIN_SATS) {
        return false;   /* 准入闸：星数太少不参与，防假定位 */
    }
    if (s->sample_ms == 0) {
        return false;   /* 从未采到 */
    }
    /* 用无符号差值比较，天然处理 tick 回绕 */
    if ((uint32_t)(now_ms - s->sample_ms) > GPS_FUSION_STALE_MS) {
        return false;   /* 样本太旧 */
    }
    return true;
}

/* 把来源的精度统一成米制 1σ 水平精度 */
static double sigma_h(const gps_sample_t *s)
{
    double a = s->h_acc_m;

    if (a <= 0.0) {
        double dop = s->pdop;
        if (dop <= 0.0) {
            /* 连 PDOP 都没有（飞控固件早于 MSP API 1.44）：只能按星数粗估。
             * 6 颗星约等于 DOP 1.0，星越多越好，夹到 [1.0, 5.0]。 */
            dop = 6.0 / (double)((s->num_sat > 0) ? s->num_sat : 1);
            if (dop < 1.0) dop = 1.0;
            if (dop > 5.0) dop = 5.0;
        }
        a = dop * GPS_FUSION_UERE_M;
    }

    if (a < GPS_FUSION_ACC_FLOOR_M) {
        a = GPS_FUSION_ACC_FLOOR_M;   /* 避免 σ→0 时权重爆炸 */
    }
    return a;
}

/* 经度加权平均：先把差值归一到 [-180,180]，否则在 ±180 附近
 * （如 179.9 与 -179.9）会被平均到 0，绕地球半圈。 */
static double blend_lon(double a, double b, double wb)
{
    double d = b - a;
    while (d > 180.0)  d -= 360.0;
    while (d < -180.0) d += 360.0;

    double r = a + d * wb;
    while (r > 180.0)  r -= 360.0;
    while (r < -180.0) r += 360.0;
    return r;
}

/* 取两者中"有效且更小"的那个（0 视为无信息） */
static double pick_min(double a, double b)
{
    if (a <= 0.0) return b;
    if (b <= 0.0) return a;
    return (a < b) ? a : b;
}

void gps_fusion_compute(const gps_sample_t *local, const gps_sample_t *fc,
                        uint32_t now_ms, gps_fused_t *out, gps_health_t *h)
{
    if (out == NULL || h == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->source = GPS_SRC_NONE;
    out->updated_ms = now_ms;

    /* 健康度按「样本本身是否可用」更新，**不受健康度自己的门限影响** ——
     * 否则一旦被判不健康就再也不会恢复（不健康 → 不参与 → 永远不累计成功）。 */
    bool raw_local = gps_sample_usable(local, now_ms);
    bool raw_fc    = gps_sample_usable(fc, now_ms);
    health_update(&h->local, raw_local);
    health_update(&h->fc,    raw_fc);

    bool ok_local = raw_local && (h->local < HEALTH_STOP_AT);
    bool ok_fc    = raw_fc    && (h->fc    < HEALTH_STOP_AT);

    if (!ok_local && !ok_fc) {
        return;   /* 没有可用来源 */
    }

    double sl = ok_local ? sigma_h(local) : 0.0;
    double sf = ok_fc    ? sigma_h(fc)    : 0.0;

    if (ok_local && ok_fc) {
        /* 逆方差加权：σ 越小权重越大（ArduPilot 的做法） */
        double wl = 1.0 / (sl * sl);
        double wf = 1.0 / (sf * sf);
        double pl = wl / (wl + wf);       /* 本地权重 */

        out->source = GPS_SRC_BOTH;
        out->lat   = local->lat   * pl + fc->lat   * (1.0 - pl);
        out->lon   = blend_lon(local->lon, fc->lon, 1.0 - pl);
        out->alt_m = local->alt_m * pl + fc->alt_m * (1.0 - pl);
        out->vel_n = local->vel_n * pl + fc->vel_n * (1.0 - pl);
        out->vel_e = local->vel_e * pl + fc->vel_e * (1.0 - pl);

        /* 垂直速度**只用本地**：飞控只给 speed + course，其合成速度的垂直分量
         * 恒为 0，平均会把真实的下降率稀释掉。 */
        out->vel_d = local->vel_d;

        out->num_sat = (local->num_sat > fc->num_sat) ? local->num_sat : fc->num_sat;
        out->h_acc_m   = pick_min(local->h_acc_m,   fc->h_acc_m);
        out->v_acc_m   = pick_min(local->v_acc_m,   fc->v_acc_m);
        out->s_acc_mps = pick_min(local->s_acc_mps, fc->s_acc_mps);
        out->pdop      = pick_min(local->pdop,      fc->pdop);

        /* 精度字段都缺时，用参与加权的 σ 兜底（至少反映相对好坏） */
        if (out->h_acc_m <= 0.0) {
            out->h_acc_m = (sl < sf) ? sl : sf;
        }
    } else {
        const gps_sample_t *s = ok_local ? local : fc;

        out->source = ok_local ? GPS_SRC_LOCAL : GPS_SRC_FC;
        out->lat   = s->lat;
        out->lon   = s->lon;
        out->alt_m = s->alt_m;
        out->vel_n = s->vel_n;
        out->vel_e = s->vel_e;
        out->vel_d = ok_local ? s->vel_d : 0.0;   /* 飞控没有垂直速度 */

        out->num_sat   = s->num_sat;
        out->h_acc_m   = (s->h_acc_m > 0.0) ? s->h_acc_m : (ok_local ? sl : sf);
        out->v_acc_m   = s->v_acc_m;
        out->s_acc_mps = s->s_acc_mps;
        out->pdop      = s->pdop;
    }

    /* 地速与航向由合成的水平速度推出，保证与 vel_n/vel_e 自洽
     * （而不是把两个来源的地速/航向各自平均——那样和位置对不上）。 */
    out->speed_ms = sqrt(out->vel_n * out->vel_n + out->vel_e * out->vel_e);
    double c = atan2(out->vel_e, out->vel_n) * DEG_PER_RAD_INV;
    if (c < 0.0) c += 360.0;
    out->course_deg = c;

    out->valid = true;
}
