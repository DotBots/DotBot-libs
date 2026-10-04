/**
 * @file
 * @brief  Host test of drv/lh2_fusion and db_pose_estimator_update_line()
 *
 * Two simulated stations cover a floor strip and overlap in the middle. The
 * robot's photodiode is projected into each station's image through the
 * inverse of its homography and quantised to LFSR counts, so the lines come
 * out of db_lh2_sweep_lines() as on the robot.
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "geometry.h"
#include "lh2_fusion.h"
#include "lh2_geometry.h"
#include "pose_estimator.h"

//=========================== harness ==========================================

static int _failed = 0;
static int _passed = 0;

#define CHECK(cond, ...)                                \
    do {                                                \
        if (cond) {                                     \
            _passed++;                                  \
        } else {                                        \
            _failed++;                                  \
            printf("FAIL %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                        \
            printf("\n");                               \
        }                                               \
    } while (0)

static const db_pose_estimator_conf_t _conf = {
    .lever_mm                     = DB_LH2_LEVER_ARM_EFFECTIVE,
    .lever_angle_deg              = DB_LH2_LEVER_ANGLE,
    .r_pos_mm2                    = DB_POSE_ESTIMATOR_R_POS_MM2,
    .q_pos_mm2_per_mm             = DB_POSE_ESTIMATOR_Q_POS_MM2_PER_MM,
    .q_heading_roll_deg2_per_mm   = DB_POSE_ESTIMATOR_Q_HEADING_ROLL_DEG2_PER_MM,
    .q_heading_turn_deg2_per_mm   = DB_POSE_ESTIMATOR_Q_HEADING_TURN_DEG2_PER_MM,
    .turn_speed_ref_mm_s          = DB_POSE_ESTIMATOR_TURN_SPEED_REF_MM_S,
    .gate                         = DB_POSE_ESTIMATOR_GATE,
    .fix_age_ticks                = 0,
    .timeout_ticks                = DB_POSE_ESTIMATOR_TIMEOUT_TICKS,
    .seed_fixes                   = DB_POSE_ESTIMATOR_SEED_FIXES,
    .seed_tolerance_mm            = DB_POSE_ESTIMATOR_SEED_TOLERANCE_MM,
    .acquire_mm                   = DB_POSE_ESTIMATOR_ACQUIRE_MM,
    .kidnap_fixes                 = DB_POSE_ESTIMATOR_KIDNAP_FIXES,
    .kidnap_still_mm              = DB_POSE_ESTIMATOR_KIDNAP_STILL_MM,
    .kidnap_settle_ticks          = DB_POSE_ESTIMATOR_KIDNAP_SETTLE_TICKS,
    .still_mm_s                   = DB_POSE_ESTIMATOR_STILL_MM_S,
    .reanchor_mm                  = DB_POSE_ESTIMATOR_REANCHOR_MM,
    .reanchor_heading_var_deg2    = DB_POSE_ESTIMATOR_REANCHOR_HEADING_VAR_DEG2,
    .q_pos_slip_mm2_per_mm_s      = DB_POSE_ESTIMATOR_Q_POS_SLIP_MM2_PER_MM_S,
    .q_heading_slip_deg2_per_mm_s = DB_POSE_ESTIMATOR_Q_HEADING_SLIP_DEG2_PER_MM_S,
    .slip_deadband_mm_s           = DB_POSE_ESTIMATOR_SLIP_DEADBAND_MM_S,
    .speed_tau_ms                 = DB_POSE_ESTIMATOR_SPEED_TAU_MS,
    .rest_mm                      = DB_POSE_ESTIMATOR_REST_MM,
    .free_spin_mm                 = DB_POSE_ESTIMATOR_FREE_SPIN_MM,
};

//=========================== simulated stations ===============================

#define STATION_A (2U)
#define STATION_B (8U)

/// Station A covers x up to about 2600 mm, station B from about 1400 mm
static const float _h_a[3][3] = { { 1300.0f, 30.0f, 1000.0f }, { -20.0f, 1300.0f, 1200.0f }, { 0.04f, 0.02f, 1.0f } };
static const float _h_b[3][3] = { { -1250.0f, 40.0f, 3000.0f }, { 30.0f, 1350.0f, 1200.0f }, { -0.03f, 0.03f, 1.0f } };

static void _invert(const float h[3][3], double inv[3][3]) {
    double a = h[0][0], b = h[0][1], c = h[0][2];
    double d = h[1][0], e = h[1][1], f = h[1][2];
    double g = h[2][0], k = h[2][1], m = h[2][2];
    double det = a * (e * m - f * k) - b * (d * m - f * g) + c * (d * k - e * g);
    inv[0][0]  = (e * m - f * k) / det;
    inv[0][1]  = (c * k - b * m) / det;
    inv[0][2]  = (b * f - c * e) / det;
    inv[1][0]  = (f * g - d * m) / det;
    inv[1][1]  = (a * m - c * g) / det;
    inv[1][2]  = (c * d - a * f) / det;
    inv[2][0]  = (d * k - e * g) / det;
    inv[2][1]  = (b * g - a * k) / det;
    inv[2][2]  = (a * e - b * d) / det;
}

static uint32_t _count(double alpha, uint8_t station) {
    return (uint32_t)llround(alpha / (2.0 * M_PI) * db_lh2_period(station) / DB_LH2_PERIOD_TICKS_PER_COUNT);
}

/// The counts station sees for a photodiode at (x, y), seen through h
/// shifted by (bias_x, bias_y) and with yaw_rad added to both sweep angles:
/// the station was moved or turned after calibration
static void _counts_for_turned(const float h[3][3], uint8_t station, double x, double y, double bias_x, double bias_y, double yaw_rad, uint32_t *c1, uint32_t *c2) {
    double inv[3][3];
    _invert(h, inv);
    x -= bias_x;
    y -= bias_y;
    double u = inv[0][0] * x + inv[0][1] * y + inv[0][2];
    double v = inv[1][0] * x + inv[1][1] * y + inv[1][2];
    double w = inv[2][0] * x + inv[2][1] * y + inv[2][2];
    u /= w;
    v /= w;
    // camera point: u = -tan(s), v = -sin(d - 60 deg) / tan(30 deg) * sqrt(1 + u^2)
    double s     = M_PI + atan(-u);
    double v_raw = v / sqrt(1.0 + u * u);
    double d     = M_PI / 3 + asin(-v_raw * tan(M_PI / 6));
    *c1          = _count(s - d + yaw_rad, station);
    *c2          = _count(s + d + yaw_rad, station);
}

static void _counts_for(const float h[3][3], uint8_t station, double x, double y, double bias_x, double bias_y, uint32_t *c1, uint32_t *c2) {
    _counts_for_turned(h, station, x, y, bias_x, bias_y, 0, c1, c2);
}

static uint8_t _lines_for(const float h[3][3], uint8_t station, double x, double y, double bias_x, double bias_y, db_lh2_floor_line_t out[2]) {
    uint32_t c1, c2;
    _counts_for(h, station, x, y, bias_x, bias_y, &c1, &c2);
    return db_lh2_sweep_lines(c1, c2, station, h, out);
}

/// Phase A's fix: the station's own solve
static void _solve(const float h[3][3], uint8_t station, double x, double y, double bias_x, double bias_y, double p[2]) {
    uint32_t c1, c2;
    _counts_for(h, station, x, y, bias_x, bias_y, &c1, &c2);
    double cam[2];
    db_lh2_camera_point(c1, c2, station, cam);
    double px = h[0][0] * cam[0] + h[0][1] * cam[1] + h[0][2];
    double py = h[1][0] * cam[0] + h[1][1] * cam[1] + h[1][2];
    double pw = h[2][0] * cam[0] + h[2][1] * cam[1] + h[2][2];
    p[0]      = px / pw;
    p[1]      = py / pw;
}

//=========================== simulated robot ==================================

typedef struct {
    float x;
    float y;
    float theta;
} robot_t;

static void _robot_step(robot_t *r, int32_t cl, int32_t cr) {
    float dl = (float)cl * DB_MM_PER_COUNT / 20.0f;
    float dr = (float)cr * DB_MM_PER_COUNT / 20.0f;
    for (int i = 0; i < 20; i++) {
        float d  = 0.5f * (dl + dr);
        float dt = -(dr - dl) / db_track_effective_mm(dl, dr);
        float m  = r->theta + 0.5f * dt;
        r->x += -d * sinf(m);
        r->y += d * cosf(m);
        r->theta += dt;
    }
}

static void _sensor(const robot_t *r, double *x, double *y) {
    *x = r->x - _conf.lever_mm * sinf(r->theta);
    *y = r->y + _conf.lever_mm * cosf(r->theta);
}

static float _sensor_error(const db_pose_estimator_t *est, const robot_t *r) {
    float  ex, ey;
    double sx, sy;
    db_pose_estimator_sensor(est, &ex, &ey);
    _sensor(r, &sx, &sy);
    return hypotf(ex - (float)sx, ey - (float)sy);
}

/// Robot at (x, y) facing +x, the estimator seeded on it
static void _start(db_pose_estimator_t *est, robot_t *r, float x, float y, float dx, float dy) {
    *r = (robot_t){ x, y, -(float)M_PI / 2 };
    db_pose_estimator_init(est, &_conf);
    db_pose_estimator_seed(est, x + dx, y + dy, -90.0f, 1.0f);
}

//=========================== tests: update_line ===============================

static void test_update_line_needs_tracking(void) {
    db_pose_estimator_t est;
    db_pose_estimator_init(&est, &_conf);
    CHECK(db_pose_estimator_update_line(&est, 1, 0, 100, 4, DB_LH2_FUSION_GATE) == DB_POSE_ESTIMATOR_REJECTED, "rejected while SEEDING");
    CHECK(est.status == DB_POSE_ESTIMATOR_SEEDING && est.chain_count == 0, "and the seed chain is untouched");
    est.status = DB_POSE_ESTIMATOR_LOST;
    CHECK(db_pose_estimator_update_line(&est, 1, 0, 100, 4, DB_LH2_FUSION_GATE) == DB_POSE_ESTIMATOR_REJECTED, "rejected while LOST");
}

static void test_update_line_moves_along_normal(void) {
    db_pose_estimator_t est;
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, 1000, 1000, 0, 0.01f);
    float sx, sy;
    db_pose_estimator_sensor(&est, &sx, &sy);
    // A line 2 mm beyond the photodiode along +x
    db_pose_estimator_result_t res = db_pose_estimator_update_line(&est, 1, 0, sx + 2.0f, 4.0f, DB_LH2_FUSION_GATE);
    float                      nx, ny;
    db_pose_estimator_sensor(&est, &nx, &ny);
    CHECK(res == DB_POSE_ESTIMATOR_ACCEPTED, "a line 2 mm off is accepted");
    CHECK(fabsf(nx - sx - 1.0f) < 0.05f && fabsf(ny - sy) < 0.05f, "equal variances split the 2 mm: moved (%.3f, %.3f)", nx - sx, ny - sy);
    CHECK(fabsf(est.last_innovation_mm - 2.0f) < 1e-4f && fabsf(est.last_line_d2 - 0.5f) < 1e-3f, "innovation 2 mm, d2 0.5: got %.4f, %.4f", est.last_innovation_mm, est.last_line_d2);
    CHECK(est.P[0][0] < 4.0f && fabsf(est.P[1][1] - 4.0f) < 1e-3f, "variance shrinks along the normal only: %.3f, %.3f", est.P[0][0], est.P[1][1]);
}

static void test_update_line_normalises(void) {
    db_pose_estimator_t a, b;
    db_pose_estimator_init(&a, &_conf);
    db_pose_estimator_init(&b, &_conf);
    db_pose_estimator_seed(&a, 500, 700, 30, 2);
    db_pose_estimator_seed(&b, 500, 700, 30, 2);
    float n = sqrtf(0.5f);
    db_pose_estimator_update_line(&a, n, n, 900, 3, DB_LH2_FUSION_GATE);
    db_pose_estimator_update_line(&b, 2 * n, 2 * n, 1800, 12, DB_LH2_FUSION_GATE);
    CHECK(fabsf(a.x - b.x) < 1e-3f && fabsf(a.y - b.y) < 1e-3f && fabsf(a.theta - b.theta) < 1e-6f, "a scaled line is the same line");
}

static void test_update_line_gate_and_degenerate(void) {
    db_pose_estimator_t est;
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, 1000, 1000, 0, 1);
    float sx, sy;
    db_pose_estimator_sensor(&est, &sx, &sy);
    float x0 = est.x;
    CHECK(db_pose_estimator_update_line(&est, 0, 1, sy + 30.0f, 4.0f, DB_LH2_FUSION_GATE) == DB_POSE_ESTIMATOR_REJECTED, "a line 30 mm off is outside the gate");
    CHECK(est.x == x0 && est.last_line_d2 > DB_LH2_FUSION_GATE, "and changes nothing but last_line_d2 (%.1f)", est.last_line_d2);
    CHECK(db_pose_estimator_update_line(&est, NAN, 1, sy, 4, DB_LH2_FUSION_GATE) == DB_POSE_ESTIMATOR_REJECTED, "NaN normal");
    CHECK(db_pose_estimator_update_line(&est, 0, 1, INFINITY, 4, DB_LH2_FUSION_GATE) == DB_POSE_ESTIMATOR_REJECTED, "infinite offset");
    CHECK(db_pose_estimator_update_line(&est, 0, 0, sy, 4, DB_LH2_FUSION_GATE) == DB_POSE_ESTIMATOR_REJECTED, "zero normal");
    CHECK(db_pose_estimator_update_line(&est, 0, 1, sy, 0, DB_LH2_FUSION_GATE) == DB_POSE_ESTIMATOR_REJECTED, "zero variance");
    CHECK(db_pose_estimator_update_line(&est, 0, 1, sy, NAN, DB_LH2_FUSION_GATE) == DB_POSE_ESTIMATOR_REJECTED, "NaN variance");
    CHECK(est.x == x0, "none of them moved the state");
}

//=========================== tests: fusion ====================================

static void test_fusion_converges_on_two_stations(void) {
    db_pose_estimator_t est;
    robot_t             r;
    db_lh2_fusion_t     fusion;
    db_lh2_fusion_init(&fusion);
    _start(&est, &r, 1800, 1500, 4, -3);
    float first = _sensor_error(&est, &r);
    for (int t = 1; t <= 300; t++) {
        db_pose_estimator_predict(&est, 0, 0, 1);
        if (t % 10 == 0) {
            double sx, sy;
            _sensor(&r, &sx, &sy);
            db_lh2_floor_line_t lines[2];
            for (int s = 0; s < 2; s++) {
                const float(*h)[3] = s ? _h_b : _h_a;
                uint8_t station    = s ? STATION_B : STATION_A;
                if (_lines_for(h, station, sx, sy, 0, 0, lines) == 2) {
                    db_lh2_fusion_update(&fusion, &est, &lines[0]);
                    db_lh2_fusion_update(&fusion, &est, &lines[1]);
                }
            }
        }
    }
    float last = _sensor_error(&est, &r);
    CHECK(first > 4.5f && last < 1.0f, "a 5 mm seed error converges under both stations: %.2f -> %.2f mm", first, last);
    CHECK(fusion.station[STATION_A].accepted == 60 && fusion.station[STATION_B].accepted == 60, "every line accepted: %u, %u", fusion.station[STATION_A].accepted, fusion.station[STATION_B].accepted);
    CHECK(fusion.station[STATION_A].rejected == 0 && fusion.station[0].accepted == 0, "nothing rejected, no other station counted");
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING && est.ticks_since_accept < 10, "lines keep the estimator tracking");
}

static void test_fusion_gates_an_outlier(void) {
    db_pose_estimator_t est;
    robot_t             r;
    db_lh2_fusion_t     fusion;
    db_lh2_fusion_init(&fusion);
    _start(&est, &r, 1800, 1500, 0, 0);
    double sx, sy;
    _sensor(&r, &sx, &sy);
    db_lh2_floor_line_t lines[2];
    _lines_for(_h_a, STATION_A, sx, sy, 0, 0, lines);
    // A reflection: the first sweep's line 150 mm away
    db_lh2_floor_line_t bad = lines[0];
    bad.d_mm += 150.0f;
    float x0 = est.x, y0 = est.y;
    CHECK(db_lh2_fusion_update(&fusion, &est, &bad) == DB_POSE_ESTIMATOR_REJECTED, "the reflected sweep is rejected");
    CHECK(est.x == x0 && est.y == y0, "and leaves the pose");
    CHECK(db_lh2_fusion_update(&fusion, &est, &lines[1]) == DB_POSE_ESTIMATOR_ACCEPTED, "while the other sweep of the pair is used");
    CHECK(fusion.station[STATION_A].accepted == 1 && fusion.station[STATION_A].rejected == 1, "health counts one of each: %u, %u", fusion.station[STATION_A].accepted, fusion.station[STATION_A].rejected);
}

static void test_fusion_not_counted_unless_tracking(void) {
    db_pose_estimator_t est;
    db_lh2_fusion_t     fusion;
    db_lh2_fusion_init(&fusion);
    db_pose_estimator_init(&est, &_conf);
    db_lh2_floor_line_t lines[2];
    _lines_for(_h_a, STATION_A, 1500, 1500, 0, 0, lines);
    CHECK(db_lh2_fusion_update(&fusion, &est, &lines[0]) == DB_POSE_ESTIMATOR_REJECTED, "rejected while SEEDING");
    CHECK(fusion.station[STATION_A].accepted == 0 && fusion.station[STATION_A].rejected == 0 && fusion.station[STATION_A].innovation_mean_mm == 0, "and not counted");
}

static void test_fusion_rejects_malformed(void) {
    db_pose_estimator_t est;
    robot_t             r;
    db_lh2_fusion_t     fusion;
    db_lh2_fusion_init(&fusion);
    _start(&est, &r, 1800, 1500, 0, 0);
    db_lh2_floor_line_t line = { .nx = 1, .ny = 0, .d_mm = 1000, .var_mm2 = 4, .station = 16 };
    CHECK(db_lh2_fusion_update(&fusion, &est, &line) == DB_POSE_ESTIMATOR_REJECTED, "station 16");
    line.station = 1;
    line.d_mm    = NAN;
    CHECK(db_lh2_fusion_update(&fusion, &est, &line) == DB_POSE_ESTIMATOR_REJECTED, "NaN offset");
    line.d_mm = 1000;
    line.nx   = 0;
    CHECK(db_lh2_fusion_update(&fusion, &est, &line) == DB_POSE_ESTIMATOR_REJECTED, "zero normal");
    line.nx      = 1;
    line.var_mm2 = -1;
    CHECK(db_lh2_fusion_update(&fusion, &est, &line) == DB_POSE_ESTIMATOR_REJECTED, "negative variance");
    CHECK(fusion.station[1].accepted == 0 && fusion.station[1].rejected == 0, "none counted");
    CHECK(fusion.gate == DB_LH2_FUSION_GATE, "default gate 10.83");
}

static void test_fusion_health_of_a_turned_station(void) {
    // B was knocked 0.3 deg round after calibration; A holds the pose, B's
    // lines arrive a quarter as often
    db_pose_estimator_t est;
    robot_t             r;
    db_lh2_fusion_t     fusion;
    db_lh2_fusion_init(&fusion);
    _start(&est, &r, 2000, 1500, 0, 0);
    for (int t = 1; t <= 2000; t++) {
        db_pose_estimator_predict(&est, 0, 0, 1);
        if (t % 10 != 0) {
            continue;
        }
        double sx, sy;
        _sensor(&r, &sx, &sy);
        db_lh2_floor_line_t lines[2];
        _lines_for(_h_a, STATION_A, sx, sy, 0, 0, lines);
        db_lh2_fusion_update(&fusion, &est, &lines[0]);
        db_lh2_fusion_update(&fusion, &est, &lines[1]);
        if (t % 40 == 0) {
            uint32_t c1, c2;
            _counts_for_turned(_h_b, STATION_B, sx, sy, 0, 0, 0.3 * M_PI / 180.0, &c1, &c2);
            db_lh2_sweep_lines(c1, c2, STATION_B, _h_b, lines);
            db_lh2_fusion_update(&fusion, &est, &lines[0]);
            db_lh2_fusion_update(&fusion, &est, &lines[1]);
        }
    }
    float mean_a = fabsf(fusion.station[STATION_A].innovation_mean_mm);
    float mean_b = fabsf(fusion.station[STATION_B].innovation_mean_mm);
    CHECK(mean_b > 2.0f && mean_b > 3.0f * mean_a, "the turned station's mean innovation drifts: B %.2f mm, A %.2f mm", mean_b, mean_a);
    CHECK(_sensor_error(&est, &r) < 3.0f, "while the pose stays near the truth: %.2f mm", _sensor_error(&est, &r));
}

static void test_fusion_counters_saturate(void) {
    db_pose_estimator_t est;
    robot_t             r;
    db_lh2_fusion_t     fusion;
    db_lh2_fusion_init(&fusion);
    _start(&est, &r, 1500, 1500, 0, 0);
    double sx, sy;
    _sensor(&r, &sx, &sy);
    db_lh2_floor_line_t lines[2];
    _lines_for(_h_a, STATION_A, sx, sy, 0, 0, lines);
    lines[0].d_mm += 500.0f;
    for (int i = 0; i < 70000; i++) {
        db_lh2_fusion_update(&fusion, &est, &lines[0]);
    }
    CHECK(fusion.station[STATION_A].rejected == UINT16_MAX && fusion.station[STATION_A].accepted == 0, "rejected saturates at 65535: %u", fusion.station[STATION_A].rejected);
}

/// Drives across the seam, 300 mm/s along +x, and returns the largest change
/// between consecutive fixes of the sensor error, for fused lines or for
/// phase A's nearest-centre fixes
static float _seam_walk(int fused, float *worst_error) {
    db_pose_estimator_t est;
    robot_t             r;
    db_lh2_fusion_t     fusion;
    db_lh2_fusion_init(&fusion);
    _start(&est, &r, 1400, 1500, 0, 0);
    // B disagrees with A by 4 mm in the overlap
    const double bias_x = 3.0, bias_y = -2.6;
    const double centre_a = 1300.0, centre_b = 2700.0;
    float        prev_ex = 0, prev_ey = 0, worst_step = 0;
    int          have_prev = 0;
    *worst_error           = 0;
    int32_t counts         = (int32_t)lroundf(3.0f / DB_MM_PER_COUNT);
    for (int t = 1; t <= 450; t++) {
        _robot_step(&r, counts, counts);
        db_pose_estimator_predict(&est, counts, counts, 1);
        if (t % 10 != 0) {
            continue;
        }
        double sx, sy;
        _sensor(&r, &sx, &sy);
        float ex, ey;
        if (fused) {
            db_lh2_floor_line_t lines[2];
            if (sx < 2600 && _lines_for(_h_a, STATION_A, sx, sy, 0, 0, lines) == 2) {
                db_lh2_fusion_update(&fusion, &est, &lines[0]);
                db_lh2_fusion_update(&fusion, &est, &lines[1]);
            }
            if (sx > 1400 && _lines_for(_h_b, STATION_B, sx, sy, bias_x, bias_y, lines) == 2) {
                db_lh2_fusion_update(&fusion, &est, &lines[0]);
                db_lh2_fusion_update(&fusion, &est, &lines[1]);
            }
            float px, py;
            db_pose_estimator_sensor(&est, &px, &py);
            ex = px - (float)sx;
            ey = py - (float)sy;
        } else {
            double pa[2], pb[2], *p;
            _solve(_h_a, STATION_A, sx, sy, 0, 0, pa);
            _solve(_h_b, STATION_B, sx, sy, bias_x, bias_y, pb);
            bool a_ok = sx < 2600;
            bool b_ok = sx > 1400;
            if (a_ok && b_ok) {
                p = (fabs(pa[0] - centre_a) <= fabs(pb[0] - centre_b)) ? pa : pb;
            } else {
                p = a_ok ? pa : pb;
            }
            ex = (float)(p[0] - sx);
            ey = (float)(p[1] - sy);
        }
        float err    = hypotf(ex, ey);
        *worst_error = fmaxf(*worst_error, err);
        if (have_prev) {
            worst_step = fmaxf(worst_step, hypotf(ex - prev_ex, ey - prev_ey));
        }
        prev_ex   = ex;
        prev_ey   = ey;
        have_prev = 1;
    }
    return worst_step;
}

/// Drives along +x at 300 mm/s under both stations, lines every 10 ticks taken
/// fix_age ticks before they reach the estimator; returns the worst sensor
/// error over the last second and the final heading error, deg
static void _drive_under_both(uint32_t fix_age, float heading_error_deg, float *worst_mm, float *heading_deg) {
    db_pose_estimator_conf_t conf = _conf;
    conf.fix_age_ticks            = fix_age;
    db_pose_estimator_t est;
    db_lh2_fusion_t     fusion;
    db_lh2_fusion_init(&fusion);
    robot_t r = { 1500, 1500, -(float)M_PI / 2 };
    db_pose_estimator_init(&est, &conf);
    db_pose_estimator_seed(&est, r.x, r.y, -90.0f + heading_error_deg, 15.0f);
    int32_t counts = (int32_t)lroundf(3.0f / DB_MM_PER_COUNT);
    double  hx[8] = { 0 }, hy[8] = { 0 };
    *worst_mm = 0;
    for (uint32_t t = 1; t <= 300; t++) {
        _robot_step(&r, counts, counts);
        db_pose_estimator_predict(&est, counts, counts, 1);
        _sensor(&r, &hx[t % 8], &hy[t % 8]);
        if (t % 10 == 0) {
            double              sx = hx[(t - fix_age) % 8], sy = hy[(t - fix_age) % 8];
            db_lh2_floor_line_t lines[2];
            for (int s = 0; s < 2; s++) {
                if (_lines_for(s ? _h_b : _h_a, s ? STATION_B : STATION_A, sx, sy, 0, 0, lines) == 2) {
                    db_lh2_fusion_update(&fusion, &est, &lines[0]);
                    db_lh2_fusion_update(&fusion, &est, &lines[1]);
                }
            }
        }
        if (t > 200) {
            *worst_mm = fmaxf(*worst_mm, _sensor_error(&est, &r));
        }
    }
    float e = fmodf(est.theta - r.theta, 2.0f * (float)M_PI);
    if (e >= (float)M_PI) {
        e -= 2.0f * (float)M_PI;
    } else if (e < -(float)M_PI) {
        e += 2.0f * (float)M_PI;
    }
    *heading_deg = fabsf(e) * 180.0f / (float)M_PI;
}

static void test_fusion_corrects_heading_while_driving(void) {
    float worst, heading;
    _drive_under_both(0, 10.0f, &worst, &heading);
    CHECK(heading < 1.5f, "lines alone pull a 10 deg heading error in while driving: %.2f deg left", heading);
    CHECK(worst < 2.0f, "and the photodiode stays on the truth: worst %.2f mm", worst);
}

static void test_fusion_corrects_heading_spinning_in_place(void) {
    // Only the lever arm makes heading observable on a spin
    db_pose_estimator_t est;
    db_lh2_fusion_t     fusion;
    db_lh2_fusion_init(&fusion);
    robot_t r = { 2000, 1500, 0 };
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 10.0f, 15.0f);
    int32_t counts = (int32_t)lroundf(1.0f / DB_MM_PER_COUNT);
    for (uint32_t t = 1; t <= 300; t++) {
        _robot_step(&r, counts, -counts);
        db_pose_estimator_predict(&est, counts, -counts, 1);
        if (t % 10 == 0) {
            double sx, sy;
            _sensor(&r, &sx, &sy);
            db_lh2_floor_line_t lines[2];
            for (int s = 0; s < 2; s++) {
                if (_lines_for(s ? _h_b : _h_a, s ? STATION_B : STATION_A, sx, sy, 0, 0, lines) == 2) {
                    db_lh2_fusion_update(&fusion, &est, &lines[0]);
                    db_lh2_fusion_update(&fusion, &est, &lines[1]);
                }
            }
        }
    }
    float e = fmodf(est.theta - r.theta, 2.0f * (float)M_PI);
    e       = (e >= (float)M_PI) ? e - 2.0f * (float)M_PI : ((e < -(float)M_PI) ? e + 2.0f * (float)M_PI : e);
    CHECK(fabsf(e) * 180.0f / (float)M_PI < 2.0f, "a spin in place under lines pulls a 10 deg heading error in: %.2f deg left", fabsf(e) * 180.0f / (float)M_PI);
}

static void test_fusion_moves_lines_by_their_age(void) {
    // 6 mm of travel per 2 ticks at 300 mm/s: an unmoved or wrongly moved line lags
    float worst, heading;
    _drive_under_both(2, 0.0f, &worst, &heading);
    CHECK(worst < 2.0f, "lines two ticks old, moved forward by the odometry: worst %.2f mm", worst);
}

static void test_fusion_seam_step(void) {
    float fused_error, nearest_error;
    float fused   = _seam_walk(1, &fused_error);
    float nearest = _seam_walk(0, &nearest_error);
    CHECK(nearest > 3.0f, "phase A steps by the stations' disagreement at the seam: %.2f mm", nearest);
    CHECK(fused < 0.5f * nearest, "fusion's step across the seam is under half of phase A's: %.2f against %.2f mm", fused, nearest);
    CHECK(fused_error < 5.0f, "and the fused estimate stays within 5 mm of the truth: %.2f mm", fused_error);
}

int main(void) {
    test_update_line_needs_tracking();
    test_update_line_moves_along_normal();
    test_update_line_normalises();
    test_update_line_gate_and_degenerate();
    test_fusion_converges_on_two_stations();
    test_fusion_gates_an_outlier();
    test_fusion_not_counted_unless_tracking();
    test_fusion_rejects_malformed();
    test_fusion_health_of_a_turned_station();
    test_fusion_counters_saturate();
    test_fusion_seam_step();
    test_fusion_corrects_heading_while_driving();
    test_fusion_corrects_heading_spinning_in_place();
    test_fusion_moves_lines_by_their_age();
    printf("lh2_fusion: %d passed, %d failed\n", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
