/**
 * @file
 * @brief  Host test of drv/lh2_geometry: the camera point and the floor lines
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "lh2_geometry.h"

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

static uint32_t _seed = 12345U;

/// Uniform in [0, 1), from a 32-bit LCG: reproducible across hosts
static double _uniform(void) {
    _seed = _seed * 1664525U + 1013904223U;
    return (double)(_seed >> 8) / 16777216.0;
}

static double _between(double lo, double hi) {
    return lo + (hi - lo) * _uniform();
}

//=========================== reference ========================================

static const uint32_t _reference_periods[16] = {
    959000, 957000, 953000, 949000, 947000, 943000, 941000, 939000,
    937000, 929000, 919000, 911000, 907000, 901000, 893000, 887000
};

/// Reference copy of the pinhole camera point formula the decoder solves with
static void _reference_camera_point(uint32_t count1, uint32_t count2, uint32_t basestation_index, double cam[2]) {
    double alpha_1 = ((double)count1 * 8.0 / _reference_periods[basestation_index]) * 2.0 * M_PI;
    double alpha_2 = ((double)count2 * 8.0 / _reference_periods[basestation_index]) * 2.0 * M_PI;

    double cam_x = -tan(0.5 * (alpha_1 + alpha_2));
    double cam_y = 0;

    if (count1 < count2) {
        cam_y = -sin(alpha_2 / 2 - alpha_1 / 2 - 60 * M_PI / 180) / tan(M_PI / 6);
    } else {
        cam_y = -sin(alpha_1 / 2 - alpha_2 / 2 - 60 * M_PI / 180) / tan(M_PI / 6);
    };
    cam_y *= sqrt(1 + cam_x * cam_x);
    cam[0] = cam_x;
    cam[1] = cam_y;
}

//=========================== helpers ==========================================

static uint32_t _count(double alpha, uint8_t station) {
    return (uint32_t)llround(alpha / (2.0 * M_PI) * db_lh2_period(station) / DB_LH2_PERIOD_TICKS_PER_COUNT);
}

/// A sweep pair seen by a station: the sweeps 2 d apart around 120 deg, either
/// order, about an azimuth s around pi or, for the other sign of cos(s), past
/// 3 pi / 2 where both angles still fit one revolution
static void _random_counts(uint8_t station, uint32_t *count1, uint32_t *count2) {
    int    near_pi = _uniform() < 0.8;
    double s       = near_pi ? _between(M_PI - 0.9, M_PI + 0.9) : _between(5.2, 5.25);
    double d       = (near_pi ? _between(40.0, 80.0) : _between(40.0, 57.0)) * M_PI / 180.0;
    double lo      = s - d;
    double hi      = s + d;
    if (_uniform() < 0.5) {
        *count1 = _count(lo, station);
        *count2 = _count(hi, station);
    } else {
        *count1 = _count(hi, station);
        *count2 = _count(lo, station);
    }
}

/// A well-conditioned homography from the image to the floor: a scale in mm,
/// a rotation, an offset and a small perspective term
static void _random_homography(float h[3][3]) {
    double k = _between(800, 2000);
    double a = _between(-M_PI, M_PI);
    h[0][0]  = (float)(k * cos(a));
    h[0][1]  = (float)(-k * sin(a) * _between(0.85, 1.0));
    h[0][2]  = (float)_between(500, 3000);
    h[1][0]  = (float)(k * sin(a));
    h[1][1]  = (float)(k * cos(a));
    h[1][2]  = (float)_between(500, 3000);
    h[2][0]  = (float)_between(-0.08, 0.08);
    h[2][1]  = (float)_between(-0.08, 0.08);
    h[2][2]  = 1.0f;
}

static void _apply(const float h[3][3], const double cam[2], double p[2]) {
    double x = h[0][0] * cam[0] + h[0][1] * cam[1] + h[0][2];
    double y = h[1][0] * cam[0] + h[1][1] * cam[1] + h[1][2];
    double w = h[2][0] * cam[0] + h[2][1] * cam[1] + h[2][2];
    p[0]     = x / w;
    p[1]     = y / w;
}

static double _residual(const db_lh2_floor_line_t *line, const double p[2]) {
    return (double)line->nx * p[0] + (double)line->ny * p[1] - (double)line->d_mm;
}

//=========================== tests ============================================

static void test_camera_point_parity(void) {
    int mismatches = 0;
    for (int i = 0; i < 20000; i++) {
        uint8_t  station = (uint8_t)(i % 16);
        uint32_t c1, c2;
        if (i % 4 == 0) {
            c1 = (uint32_t)(_uniform() * db_lh2_period(station) / 8);
            c2 = (uint32_t)(_uniform() * db_lh2_period(station) / 8);
        } else {
            _random_counts(station, &c1, &c2);
        }
        double got[2], want[2];
        db_lh2_camera_point(c1, c2, station, got);
        _reference_camera_point(c1, c2, station, want);
        if (got[0] != want[0] || got[1] != want[1]) {
            mismatches++;
        }
    }
    CHECK(mismatches == 0, "camera point bit-identical to the pre-move formula: %d of 20000 differ", mismatches);
}

static void test_camera_point_out_of_range(void) {
    double cam[2];
    db_lh2_camera_point(1000, 2000, 16, cam);
    CHECK(isnan(cam[0]) && isnan(cam[1]), "station 16 gives NaN: got (%f, %f)", cam[0], cam[1]);
    CHECK(db_lh2_period(16) == 0 && db_lh2_period(0) == 959000, "period table: %u, %u", db_lh2_period(16), db_lh2_period(0));
}

static void test_lines_meet_at_the_solve(void) {
    double worst  = 0;
    int    failed = 0;
    for (int i = 0; i < 5000; i++) {
        uint8_t  station = (uint8_t)(i % 16);
        uint32_t c1, c2;
        _random_counts(station, &c1, &c2);
        float h[3][3];
        _random_homography(h);
        db_lh2_floor_line_t lines[2];
        if (db_lh2_sweep_lines(c1, c2, station, (const float(*)[3])h, lines) != 2) {
            failed++;
            continue;
        }
        double cam[2], p[2];
        db_lh2_camera_point(c1, c2, station, cam);
        _apply(h, cam, p);
        for (int k = 0; k < 2; k++) {
            double r = fabs(_residual(&lines[k], p));
            worst    = (r > worst) ? r : worst;
        }
        // The two lines' own intersection
        double det = (double)lines[0].nx * lines[1].ny - (double)lines[0].ny * lines[1].nx;
        if (fabs(det) > 1e-3) {
            double x = ((double)lines[0].d_mm * lines[1].ny - (double)lines[1].d_mm * lines[0].ny) / det;
            double y = ((double)lines[0].nx * lines[1].d_mm - (double)lines[1].nx * lines[0].d_mm) / det;
            double e = hypot(x - p[0], y - p[1]);
            worst    = (e > worst) ? e : worst;
        }
    }
    CHECK(failed == 0, "every well-conditioned pair gives two lines: %d failed", failed);
    CHECK(worst < 0.01, "the two lines meet at H(camera point) within 0.01 mm: worst %.5f mm", worst);
}

static void test_line_is_one_sweep(void) {
    // Sweep 0 is the level set of count1: moving count2 leaves its line, and
    // moves the other one
    double worst_same = 0, least_moved = 1e9;
    for (int i = 0; i < 2000; i++) {
        uint8_t  station = (uint8_t)(i % 16);
        uint32_t c1, c2;
        _random_counts(station, &c1, &c2);
        float h[3][3];
        _random_homography(h);
        int32_t             step = (c1 < c2) ? 200 : -200;
        db_lh2_floor_line_t a[2], b[2], c[2];
        if (db_lh2_sweep_lines(c1, c2, station, (const float(*)[3])h, a) != 2 || db_lh2_sweep_lines(c1, (uint32_t)((int32_t)c2 + step), station, (const float(*)[3])h, b) != 2 || db_lh2_sweep_lines((uint32_t)((int32_t)c1 - step), c2, station, (const float(*)[3])h, c) != 2) {
            continue;
        }
        double sign0 = (a[0].nx * b[0].nx + a[0].ny * b[0].ny < 0) ? -1.0 : 1.0;
        double same0 = fabs(a[0].d_mm - sign0 * b[0].d_mm) + 1000.0 * (fabs(a[0].nx - sign0 * b[0].nx) + fabs(a[0].ny - sign0 * b[0].ny));
        double sign1 = (a[1].nx * c[1].nx + a[1].ny * c[1].ny < 0) ? -1.0 : 1.0;
        double same1 = fabs(a[1].d_mm - sign1 * c[1].d_mm) + 1000.0 * (fabs(a[1].nx - sign1 * c[1].nx) + fabs(a[1].ny - sign1 * c[1].ny));
        worst_same   = fmax(worst_same, fmax(same0, same1));
        double moved = fabs(a[1].d_mm - b[1].d_mm) + 1000.0 * (fabs(a[1].nx - b[1].nx) + fabs(a[1].ny - b[1].ny));
        least_moved  = fmin(least_moved, moved);
    }
    CHECK(worst_same < 0.05, "a line stays put when only the other sweep's count changes: worst %.4f", worst_same);
    CHECK(least_moved > 0.5, "and the line of the sweep whose count changed moves: least %.4f", least_moved);
}

static void test_variance_from_sweep_sigma(void) {
    // Raising count1 by a few counts moves the solve along sweep 0's normal,
    // forwards, by sqrt(var) / sigma per radian
    double worst = 0;
    for (int i = 0; i < 2000; i++) {
        uint8_t  station = (uint8_t)(i % 16);
        uint32_t c1, c2;
        _random_counts(station, &c1, &c2);
        float h[3][3];
        _random_homography(h);
        db_lh2_floor_line_t lines[2];
        if (db_lh2_sweep_lines(c1, c2, station, (const float(*)[3])h, lines) != 2) {
            continue;
        }
        double cam0[2], cam1[2], p0[2], p1[2];
        db_lh2_camera_point(c1, c2, station, cam0);
        db_lh2_camera_point(c1 + 3, c2, station, cam1);
        _apply(h, cam0, p0);
        _apply(h, cam1, p1);
        double moved    = lines[0].nx * (p1[0] - p0[0]) + lines[0].ny * (p1[1] - p0[1]);
        double dalpha   = 3.0 * DB_LH2_PERIOD_TICKS_PER_COUNT / db_lh2_period(station) * 2.0 * M_PI;
        double expected = sqrt(lines[0].var_mm2) / DB_LH2_SWEEP_SIGMA_RAD * dalpha;
        double ratio    = moved / expected;
        worst           = fmax(worst, fabs(ratio - 1.0));
    }
    CHECK(worst < 0.05, "the variance is the sweep sigma carried through H, the normal points the way the sweep moves: worst ratio error %.4f", worst);
}

static void test_typical_variance(void) {
    // An image-to-floor scale of 1500 mm per unit, looking straight down
    float               h[3][3] = { { 1500, 0, 2000 }, { 0, 1500, 2000 }, { 0, 0, 1 } };
    db_lh2_floor_line_t lines[2];
    uint32_t            c1 = _count(M_PI - M_PI / 3, 0);
    uint32_t            c2 = _count(M_PI + M_PI / 3, 0);
    CHECK(db_lh2_sweep_lines(c1, c2, 0, (const float(*)[3])h, lines) == 2, "lines at the centre of the view");
    float sd0 = sqrtf(lines[0].var_mm2), sd1 = sqrtf(lines[1].var_mm2);
    CHECK(sd0 > 0.5f && sd0 < 5.0f && sd1 > 0.5f && sd1 < 5.0f, "a few mm along the normal at the centre of the view: %.3f, %.3f", sd0, sd1);
    CHECK(lines[0].station == 0 && lines[0].sweep == 0 && lines[1].station == 0 && lines[1].sweep == 1, "station and sweep fields");
    CHECK(fabsf(lines[0].nx * lines[0].nx + lines[0].ny * lines[0].ny - 1.0f) < 1e-5f, "unit normal");
}

static void test_degenerate(void) {
    float               singular[3][3] = { { 1000, 2000, 0 }, { 500, 1000, 0 }, { 0, 0, 0 } };
    float               nan_h[3][3]    = { { NAN, 0, 0 }, { 0, 1000, 0 }, { 0, 0, 1 } };
    float               ok[3][3]       = { { 1500, 0, 2000 }, { 0, 1500, 2000 }, { 0, 0, 1 } };
    db_lh2_floor_line_t lines[2]       = { 0 };
    uint32_t            c1             = _count(M_PI - 1.0, 3);
    uint32_t            c2             = _count(M_PI + 1.0, 3);
    CHECK(db_lh2_sweep_lines(c1, c2, 3, (const float(*)[3])singular, lines) == 0, "singular homography");
    CHECK(db_lh2_sweep_lines(c1, c2, 3, (const float(*)[3])nan_h, lines) == 0, "NaN homography");
    CHECK(db_lh2_sweep_lines(c1, c2, 16, (const float(*)[3])ok, lines) == 0, "station out of range");
    // Azimuth at a right angle: the image point is at infinity
    uint32_t q1 = _count(M_PI / 2 - 1.0, 0);
    uint32_t q2 = _count(M_PI / 2 + 1.0, 0);
    uint8_t  n  = db_lh2_sweep_lines(q1, q2, 0, (const float(*)[3])ok, lines);
    CHECK(n == 0 || (isfinite(lines[0].d_mm) && isfinite(lines[1].d_mm)), "an azimuth at 90 deg is degenerate or finite, never NaN out");
}

int main(void) {
    test_camera_point_parity();
    test_camera_point_out_of_range();
    test_lines_meet_at_the_solve();
    test_line_is_one_sweep();
    test_variance_from_sweep_sigma();
    test_typical_variance();
    test_degenerate();
    printf("lh2_geometry: %d passed, %d failed\n", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
