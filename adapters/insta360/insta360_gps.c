/* SPDX-License-Identifier: MIT */

#include "insta360_gps.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "gps.h"
#include "gps_fusion.h"
#include "insta360.h"

/* 帧格式（抓包实证）：FC EF FE 83 <len16> ,<海拔%.1f>,<星数 裸字节>,$GNRMC…*<CS>
 * 星数是单字节不是 ASCII；长度按内容算；RMC 时间是 UTC（本地 GNSS 给的就是 UTC）。 */

static int fmt_coord(char *out, size_t n, double deg, int w)
{
    char hemi = (w == 3) ? ((deg < 0) ? 'W' : 'E') : ((deg < 0) ? 'S' : 'N');
    double a = fabs(deg);
    int d = (int)a;
    double m = (a - d) * 60.0;
    if (m >= 59.9995) { d += 1; m = 0.0; }
    return snprintf(out, n, "%0*d%07.4f,%c", w, d, m, hemi);
}

static int build_rmc(char *out, size_t n, const gps_fused_t *f, const GPS_Data_t *l)
{
    char lat[16], lon[16];
    fmt_coord(lat, sizeof(lat), f->lat, 2);
    fmt_coord(lon, sizeof(lon), f->lon, 3);

    int len = snprintf(out, n,
        "$GNRMC,%02u%02u%02u.000,A,%s,%s,%.2f,%.2f,%02u%02u%02u,,,D,V*",
        (unsigned)l->Hour, (unsigned)l->Minute, (unsigned)l->Second,
        lat, lon, f->speed_ms * 1.9438444924406, f->course_deg,
        (unsigned)l->Day, (unsigned)l->Month, (unsigned)(l->Year % 100));

    uint8_t cs = 0;
    for (int i = 1; i < len - 1; i++) cs ^= (uint8_t)out[i];
    out[len]     = "0123456789ABCDEF"[cs >> 4];
    out[len + 1] = "0123456789ABCDEF"[cs & 0xF];
    out[len + 2] = '\0';
    return len + 2;
}

static void insta360_gps_push(void)
{
    if (!insta360_is_connected() || !insta360_is_recording()) return;

    gps_fused_t f;
    gps_fusion_get(&f);
    if (!f.valid) return;

    GPS_Data_t l;
    if (!gps_logic_snapshot(&l)) return;

    char rmc[96];
    int r = build_rmc(rmc, sizeof(rmc), &f, &l);

    uint8_t frame[128];
    int p = 6;
    p += snprintf((char *)&frame[p], sizeof(frame) - p, ",%.1f,", f.alt_m);
    frame[p++] = (uint8_t)f.num_sat;
    frame[p++] = ',';
    memcpy(&frame[p], rmc, r);
    p += r;

    frame[0] = 0xFC; frame[1] = 0xEF; frame[2] = 0xFE; frame[3] = 0x83;
    frame[4] = (uint8_t)((p - 6) >> 8);
    frame[5] = (uint8_t)((p - 6) & 0xFF);

    insta360_send_raw(frame, (size_t)p);
}

void insta360_gps_init(void)
{
    gps_fusion_set_ready_cb(insta360_gps_push);
}
