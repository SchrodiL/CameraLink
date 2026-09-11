/* SPDX-License-Identifier: MIT */

#ifndef GPS_FUSION_MATH_H
#define GPS_FUSION_MATH_H

#include <stdint.h>
#include <stdbool.h>

/*
 * gps_fusion_math.h —— GPS 双源融合的**纯算法**部分。
 *
 * 这个文件刻意不依赖 ESP-IDF / FreeRTOS，只做算术，便于在主机上单测
 * （见 test/gps_fusion/）。运行时状态、加锁、跨任务发布都在 gps_fusion.c。
 *
 * 算法参考 ArduPilot 的 AP_GPS_Blended（双 GPS 融合）：
 *   - 权重取**逆方差**：w ∝ 1/σ²，归一化。精度好的权重大。
 *   - 位置/速度按权重平均。
 *   - 精度 / DOP 取两者**最优（最小）**；星数取**最大**。
 *   - 只有一个来源可用时直接透传，不做平均（避免"用单源模拟出中间值"）。
 *   - 健康计数做滞回，避免来源在边界上反复进出融合。
 */

/* 一个 GPS 来源的样本（已归一化到统一单位） */
typedef struct {
    bool     valid;          /* 该来源本次是否有可用定位 */
    uint8_t  fix_type;       /* 0=none, 2=2D, 3=3D */
    uint8_t  num_sat;        /* 卫星数 */
    double   lat, lon;       /* 度 */
    double   alt_m;          /* 海拔（MSL）米 */
    double   vel_n, vel_e, vel_d;   /* m/s，vel_d 向下为正 */
    double   course_deg;     /* 航向，度 */
    double   h_acc_m;        /* 水平精度 1σ（米）；0 = 未知 */
    double   v_acc_m;        /* 垂直精度 1σ（米）；0 = 未知 */
    double   s_acc_mps;      /* 速度精度 1σ（m/s）；0 = 未知 */
    double   pdop;           /* 位置精度因子；0 = 未知 */
    uint32_t sample_ms;      /* 采样时刻（同源时钟），0 = 从未采到 */
} gps_sample_t;

typedef enum {
    GPS_SRC_NONE  = 0,   /* 两个来源都没有定位 */
    GPS_SRC_LOCAL = 1,   /* 只用板载 GNSS */
    GPS_SRC_FC    = 2,   /* 只用飞控的 GPS */
    GPS_SRC_BOTH  = 3,   /* 两者融合 */
} gps_src_t;

typedef struct {
    bool     valid;
    gps_src_t source;
    uint8_t  num_sat;        /* 两个可用来源里取最大 */
    double   lat, lon, alt_m;
    double   vel_n, vel_e, vel_d;
    double   speed_ms;       /* 由 vel_n/vel_e 合成 */
    double   course_deg;
    double   h_acc_m, v_acc_m, s_acc_mps;   /* 取最优（最小） */
    double   pdop;
    uint32_t updated_ms;
} gps_fused_t;

/* 每个来源的健康计数（滞回）。由 gps_fusion_compute 就地更新。 */
typedef struct {
    uint8_t local;
    uint8_t fc;
} gps_health_t;

/* ---- 可调参数 ---- */

/* PDOP → 米制 1σ 的换算系数。真实 UERE 随电离层/卫星仰角变化，1.5 是个
 * 经验值；它只影响两个来源之间的**相对**权重，绝对值不敏感。 */
#define GPS_FUSION_UERE_M        1.5

/* 低于这个星数不参与融合。用来挡"假定位"——hAcc/PDOP 本身已含卫星几何，
 * 星数再乘进权重是重复计权，所以这里是**准入闸**而不是权重系数。 */
#define GPS_FUSION_MIN_SATS      5

/* 样本超过这个年龄就丢弃（毫秒）。飞控那边由 osd_task 每 100ms 轮询更新，
 * 本地由 GNSS 每秒多次更新，1.5s 足够宽松又不会用上卡死的旧值。 */
#define GPS_FUSION_STALE_MS      1500

/* 精度下限，避免 σ→0 时权重爆炸 */
#define GPS_FUSION_ACC_FLOOR_M   0.3

/*
 * 样本本身是否可用：有 3D 定位、星数过闸、且没超过 STALE_MS。
 * **不含**健康度判定 —— 健康度是运行时累积的状态，由 gps_fusion_compute 处理。
 * 单独暴露出来是为了让上层能分别显示"哪一路有数据/有定位"（状态灯用）。
 */
bool gps_sample_usable(const gps_sample_t *s, uint32_t now_ms);

/*
 * 计算融合结果。
 *   local / fc  —— 两个来源的样本（可以为 NULL 或 valid=false）
 *   now_ms      —— 当前时刻（与 sample_ms 同源）
 *   out         —— 输出（总是被写入；没有定位时 out->valid = false）
 *   h           —— 健康计数，**就地更新**（调用方持有，跨周期累积）
 */
void gps_fusion_compute(const gps_sample_t *local, const gps_sample_t *fc,
                        uint32_t now_ms, gps_fused_t *out, gps_health_t *h);

#endif /* GPS_FUSION_MATH_H */
