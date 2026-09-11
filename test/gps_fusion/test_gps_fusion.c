/*
 * GPS 双源融合算法的主机测试。
 *
 * 只测 gps_fusion_math.c（纯算术、无 ESP-IDF 依赖）—— 加锁、跨任务发布那些
 * 在硬件上验证，这里把算法逻辑钉死，免得改参数时悄悄改坏行为。
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "gps_fusion_math.h"

static int checks = 0, fails = 0;

#define CHECK(cond, msg) do { \
        checks++; \
        if (!(cond)) { fails++; printf("  FAIL  %s   (%s:%d)\n", msg, __FILE__, __LINE__); } \
    } while (0)

#define NEAR(a, b, tol) (fabs((a) - (b)) <= (tol))

/* 造一个样本：h_acc 为 0 表示"没有米制精度"（飞控侧就是这样） */
static gps_sample_t mksample(double lat, double lon, double hacc, double pdop,
                             uint8_t sats, uint32_t t)
{
    gps_sample_t s;
    memset(&s, 0, sizeof(s));
    s.valid     = true;
    s.fix_type  = 3;
    s.num_sat   = sats;
    s.lat       = lat;
    s.lon       = lon;
    s.alt_m     = 100.0;
    s.vel_n     = 3.0;      /* 向北 3 m/s */
    s.vel_e     = 4.0;      /* 向东 4 m/s → 地速应为 5 m/s */
    s.vel_d     = 0.5;      /* 下降 0.5 m/s */
    s.h_acc_m   = hacc;
    s.v_acc_m   = hacc * 2.0;
    s.s_acc_mps = 1.0;
    s.pdop      = pdop;
    s.sample_ms = t;
    return s;
}

static void case_local_only(void)
{
    gps_sample_t local = mksample(39.0, 117.0, 1.0, 0, 10, 1000);
    gps_fused_t out; gps_health_t h = {0, 0};

    gps_fusion_compute(&local, NULL, 1000, &out, &h);

    printf("本地独有\n");
    CHECK(out.valid, "应有效");
    CHECK(out.source == GPS_SRC_LOCAL, "来源应为 LOCAL");
    CHECK(NEAR(out.lat, 39.0, 1e-9), "纬度应透传");
    CHECK(NEAR(out.speed_ms, 5.0, 1e-6), "地速应由 vel_n/vel_e 合成 = 5");
    CHECK(NEAR(out.course_deg, 53.130102, 1e-4), "航向应为 atan2(4,3)");
}

static void case_fc_only(void)
{
    /* 飞控只有 PDOP，没有米制精度 */
    gps_sample_t fc = mksample(-33.0, 151.0, 0.0, 1.2, 12, 2000);
    gps_fused_t out; gps_health_t h = {0, 0};

    gps_fusion_compute(NULL, &fc, 2000, &out, &h);

    printf("飞控独有\n");
    CHECK(out.source == GPS_SRC_FC, "来源应为 FC");
    CHECK(NEAR(out.lat, -33.0, 1e-9), "纬度应透传");
    CHECK(NEAR(out.vel_d, 0.0, 1e-9), "飞控无垂直速度，vel_d 应为 0");
    CHECK(out.h_acc_m > 0.0, "无米制精度时应回落成 PDOP×UERE（>0）");
}

static void case_both_equal_weight(void)
{
    /* 两边精度相同 → 等权 → 正好中点 */
    gps_sample_t local = mksample(39.0000, 117.0000, 5.0, 0, 10, 3000);
    gps_sample_t fc    = mksample(39.0020, 117.0040, 0.0, 5.0 / GPS_FUSION_UERE_M, 10, 3000);
    gps_fused_t out; gps_health_t h = {0, 0};

    gps_fusion_compute(&local, &fc, 3000, &out, &h);

    printf("两者等精度\n");
    CHECK(out.source == GPS_SRC_BOTH, "来源应为 BOTH");
    CHECK(NEAR(out.lat, 39.0010, 1e-6), "等权应取中点");
    CHECK(NEAR(out.lon, 117.0020, 1e-6), "等权应取中点");
    CHECK(out.num_sat == 10, "星数取最大");
    CHECK(NEAR(out.vel_d, 0.5, 1e-9), "vel_d 应只取本地（不被稀释成一半）");
}

static void case_weight_favours_better(void)
{
    /* 本地 0.5m vs 飞控 10m（PDOP≈6.67）→ 结果应明显偏向本地 */
    gps_sample_t local = mksample(39.0000, 117.0, 0.5, 0, 14, 4000);
    gps_sample_t fc    = mksample(39.0100, 117.0, 0.0, 10.0 / GPS_FUSION_UERE_M, 6, 4000);
    gps_fused_t out; gps_health_t h = {0, 0};

    gps_fusion_compute(&local, &fc, 4000, &out, &h);

    printf("一好一坏（权重应偏向好的）\n");
    CHECK(out.lat < 39.0010, "精度好的一侧应主导（不能是 39.005 那种中点）");
    CHECK(out.lat > 39.0000, "但仍应被差的一侧拉动一点点");
}

static void case_min_max_rules(void)
{
    gps_sample_t local = mksample(39.0, 117.0, 2.0, 1.0, 8,  5000);
    gps_sample_t fc    = mksample(39.0, 117.0, 1.0, 0.8, 15, 5000);
    gps_fused_t out; gps_health_t h = {0, 0};

    gps_fusion_compute(&local, &fc, 5000, &out, &h);

    printf("最小/最大规则\n");
    CHECK(NEAR(out.h_acc_m, 1.0, 1e-9), "水平精度取最优（最小）");
    CHECK(NEAR(out.pdop, 0.8, 1e-9), "PDOP 取最优（最小）");
    CHECK(out.num_sat == 15, "星数取最大");
}

static void case_gates(void)
{
    gps_sample_t good = mksample(39.0, 117.0, 1.0, 0, 10, 6000);
    gps_fused_t out; gps_health_t h = {0, 0};

    printf("准入门槛\n");

    /* 星数不足 */
    gps_sample_t few = mksample(39.0, 117.0, 1.0, 0, GPS_FUSION_MIN_SATS - 1, 6000);
    gps_fusion_compute(&few, NULL, 6000, &out, &h);
    CHECK(!out.valid, "星数 < 门槛 应不参与");

    /* 只有 2D 定位 */
    gps_sample_t d2 = mksample(39.0, 117.0, 1.0, 0, 10, 6000);
    d2.fix_type = 2;
    h.local = h.fc = 0;
    gps_fusion_compute(&d2, NULL, 6000, &out, &h);
    CHECK(!out.valid, "2D 定位应不参与");

    /* 样本过旧 */
    h.local = h.fc = 0;
    gps_fusion_compute(&good, NULL, 6000 + GPS_FUSION_STALE_MS + 1, &out, &h);
    CHECK(!out.valid, "过旧的样本应被丢弃");

    /* 恰好还没过期 */
    h.local = h.fc = 0;
    gps_fusion_compute(&good, NULL, 6000 + GPS_FUSION_STALE_MS, &out, &h);
    CHECK(out.valid, "恰好等于门槛应仍然有效");
}

static void case_hysteresis(void)
{
    gps_sample_t bad  = mksample(39.0, 117.0, 1.0, 0, 3, 7000);   /* 星数不足 = 不可用 */
    gps_sample_t good = mksample(39.0, 117.0, 1.0, 0, 10, 7000);
    gps_fused_t out; gps_health_t h = {0, 0};

    printf("健康度滞回\n");

    /* 连续失败 10 次 → 计数应封顶 100 */
    for (int i = 0; i < 10; i++) {
        gps_fusion_compute(&bad, NULL, 7000, &out, &h);
    }
    CHECK(h.local == 100, "连续失败应爬到上限 100");
    CHECK(!out.valid, "不健康期间不参与");

    /* 转好之后：计数每次 −1，降到 50 以下才重新参与。
     * 从 100 降起，前 50 次成功仍 >= 50（被排除），第 51 次才 < 50。 */
    for (int i = 0; i < 50; i++) {
        gps_fusion_compute(&good, NULL, 7000, &out, &h);
    }
    CHECK(h.local == 50, "50 次成功后计数应为 50");
    CHECK(!out.valid, "计数 >= 50 时仍应排除（这就是滞回）");

    gps_fusion_compute(&good, NULL, 7000, &out, &h);
    CHECK(h.local == 49, "第 51 次成功应降到 49");
    CHECK(out.valid, "计数 < 50 后应重新参与");
}

static void case_longitude_wrap(void)
{
    /* 179.9 与 -179.9 实际只差 0.2 度，平均应落在 ±180 附近而不是 0 */
    gps_sample_t local = mksample(0.0,  179.9, 5.0, 0, 10, 8000);
    gps_sample_t fc    = mksample(0.0, -179.9, 5.0, 0, 10, 8000);
    gps_fused_t out; gps_health_t h = {0, 0};

    gps_fusion_compute(&local, &fc, 8000, &out, &h);

    printf("经度跨 ±180\n");
    CHECK(fabs(out.lon) > 179.0, "应落在 ±180 附近（不能被平均到 0 附近）");
}

int main(void)
{
    printf("=== GPS 融合算法测试 ===\n");
    case_local_only();
    case_fc_only();
    case_both_equal_weight();
    case_weight_favours_better();
    case_min_max_rules();
    case_gates();
    case_hysteresis();
    case_longitude_wrap();

    printf("\n%d 项检查，%d 项失败\n", checks, fails);
    return (fails == 0) ? 0 : 1;
}
