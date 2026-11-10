/**
 * @file
 * @brief  Synthetic LH2 scenes for the host tests
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "lh2_geometry.h"
#include "lh2_scene.h"

//=========================== private ==========================================

static bool _invert(const float h[3][3], double inv[3][3]) {
    double a = h[0][0], b = h[0][1], c = h[0][2];
    double d = h[1][0], e = h[1][1], f = h[1][2];
    double g = h[2][0], k = h[2][1], m = h[2][2];
    double det = a * (e * m - f * k) - b * (d * m - f * g) + c * (d * k - e * g);
    if (!isfinite(det) || fabs(det) < 1e-12) {
        return false;
    }
    inv[0][0] = (e * m - f * k) / det;
    inv[0][1] = (c * k - b * m) / det;
    inv[0][2] = (b * f - c * e) / det;
    inv[1][0] = (f * g - d * m) / det;
    inv[1][1] = (a * m - c * g) / det;
    inv[1][2] = (c * d - a * f) / det;
    inv[2][0] = (d * k - e * g) / det;
    inv[2][1] = (b * g - a * k) / det;
    inv[2][2] = (a * e - b * d) / det;
    return true;
}

static void _at(const double from[2], const double to[2], double from_us, double to_us, double t_us, double xy[2]) {
    double f = (to_us > from_us) ? (t_us - from_us) / (to_us - from_us) : 0.0;
    f        = (f < 0) ? 0 : ((f > 1) ? 1 : f);
    xy[0]    = from[0] + f * (to[0] - from[0]);
    xy[1]    = from[1] + f * (to[1] - from[1]);
}

/// The data bit of one revolution, the same whichever window asks
static uint8_t _data_bit(uint8_t station, int64_t revolution) {
    uint32_t hsh = (uint32_t)revolution * 2654435761U ^ (uint32_t)station * 40503U;
    hsh ^= hsh >> 15;
    hsh *= 2246822519U;
    return (uint8_t)(hsh >> 31);
}

static int _by_time(const void *a, const void *b) {
    double ta = ((const lh2_synth_event_t *)a)->t_true_us;
    double tb = ((const lh2_synth_event_t *)b)->t_true_us;
    return (ta > tb) - (ta < tb);
}

//=========================== public ===========================================

bool lh2_scene_station_init(lh2_scene_station_t *st, uint8_t station, double cx_mm, double cy_mm, double k_mm, double yaw_rad, double pu, double pv, double t0_us) {
    memset(st, 0, sizeof(*st));
    st->station = station;
    st->t0_us   = t0_us;
    st->h[0][0] = (float)(k_mm * cos(yaw_rad));
    st->h[0][1] = (float)(-k_mm * sin(yaw_rad));
    st->h[0][2] = (float)cx_mm;
    st->h[1][0] = (float)(k_mm * sin(yaw_rad));
    st->h[1][1] = (float)(k_mm * cos(yaw_rad));
    st->h[1][2] = (float)cy_mm;
    st->h[2][0] = (float)pu;
    st->h[2][1] = (float)pv;
    st->h[2][2] = 1.0f;
    return _invert((const float(*)[3])st->h, st->h_inv);
}

void lh2_scene_camera_point(const lh2_scene_station_t *st, double x_mm, double y_mm, double cam[2]) {
    double u = st->h_inv[0][0] * x_mm + st->h_inv[0][1] * y_mm + st->h_inv[0][2];
    double v = st->h_inv[1][0] * x_mm + st->h_inv[1][1] * y_mm + st->h_inv[1][2];
    double w = st->h_inv[2][0] * x_mm + st->h_inv[2][1] * y_mm + st->h_inv[2][2];
    cam[0]   = u / w;
    cam[1]   = v / w;
}

bool lh2_scene_counts(uint8_t station, const double cam[2], double counts[2]) {
    double period = db_lh2_period(station);
    if (!(period > 0) || !isfinite(cam[0]) || !isfinite(cam[1])) {
        return false;
    }
    double half_sum      = atan(-cam[0]);
    double tan_elevation = cam[1] / sqrt(1 + cam[0] * cam[0]);
    double s             = -tan_elevation * tan(M_PI / 6);
    if (!(fabs(s) <= 1.0)) {
        return false;
    }
    double half_diff = M_PI / 3 + asin(s);
    double a1        = half_sum - half_diff;
    double a2        = half_sum + half_diff;
    while (a1 < 0) {
        a1 += M_PI;
        a2 += M_PI;
    }
    double scale = period / DB_LH2_PERIOD_TICKS_PER_COUNT / (2 * M_PI);
    counts[0]    = a1 * scale;
    counts[1]    = a2 * scale;
    return counts[1] < period / DB_LH2_PERIOD_TICKS_PER_COUNT;
}

size_t lh2_scene_window(const lh2_scene_station_t *stations, uint8_t n_stations, double from_us, double to_us, const double xy_from[2], const double xy_to[2], uint64_t base_us, lh2_synth_rng_t *rng, lh2_synth_event_t *out, size_t max) {
    size_t n = 0;
    for (uint8_t i = 0; i < n_stations; i++) {
        const lh2_scene_station_t *st     = &stations[i];
        double                     period = lh2_synth_period_us(st->station);
        int64_t                    first  = (int64_t)floor((from_us - st->t0_us) / period) - 1;
        for (int64_t rev = (first < 0) ? 0 : first;; rev++) {
            double t_rev = st->t0_us + (double)rev * period;
            if (t_rev >= to_us) {
                break;
            }
            uint8_t polynomial = (uint8_t)(2U * st->station + _data_bit(st->station, rev));
            for (uint8_t sweep = 0; sweep < 2; sweep++) {
                // The sweep's time depends on where the photodiode is then: two passes settle it
                double xy[2], cam[2], counts[2];
                double t  = t_rev;
                bool   ok = true;
                for (uint8_t pass = 0; pass < 2 && ok; pass++) {
                    _at(xy_from, xy_to, from_us, to_us, t, xy);
                    lh2_scene_camera_point(st, xy[0], xy[1], cam);
                    ok = lh2_scene_counts(st->station, cam, counts);
                    t  = t_rev + counts[sweep] / LH2_SYNTH_COUNTS_PER_US;
                }
                if (!ok || t < from_us || t >= to_us || (t >= st->silent_us[0] && t < st->silent_us[1])) {
                    continue;
                }
                bool   dup   = ((st->dup_sweeps >> sweep) & 1U) && st->dup_share > 0 && lh2_synth_uniform(rng) < st->dup_share;
                double delay = st->dup_delay_us[0] + (st->dup_delay_us[1] - st->dup_delay_us[0]) * lh2_synth_uniform(rng);
                for (uint8_t copy = 0; copy < (dup ? 2 : 1); copy++) {
                    if (n == max) {
                        return max;
                    }
                    lh2_synth_event_t *ev    = &out[n++];
                    double             c     = counts[sweep] + (copy ? delay * LH2_SYNTH_COUNTS_PER_US : 0);
                    double             t_cap = t + (copy ? delay : 0);
                    uint32_t           n0    = (uint32_t)floor(c);
                    lh2_synth_light_t  light = {
                         .polynomial = polynomial,
                         .count      = n0,
                         .phase      = (c - n0) * LH2_SYNTH_SAMPLES_PER_BIT,
                         .level      = (uint8_t)(lh2_synth_rand(rng) >> 31),
                         .idle_level = (uint8_t)(lh2_synth_rand(rng) >> 31),
                    };
                    lh2_synth_capture(&light, rng, ev->spi);
                    double latency = st->latency_us * lh2_synth_uniform(rng);
                    ev->t_us       = (uint32_t)(base_us + (uint64_t)floor(t_cap + LH2_SYNTH_CAPTURE_US + latency));
                    ev->t_true_us  = t_cap;
                    ev->station    = st->station;
                    ev->sweep      = sweep;
                    ev->polynomial = polynomial;
                    ev->count      = n0;
                    ev->revolution = (uint32_t)rev;
                    ev->duplicate  = copy == 1;
                }
            }
        }
    }
    qsort(out, n, sizeof(out[0]), _by_time);
    return n;
}
