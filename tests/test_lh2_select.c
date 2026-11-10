/**
 * @file
 * @brief  Host test of drv/lh2_select against the frozen swarmit a702520 choice
 *
 * The golden comparison: random calibrations and random ready pairs go to
 * both the reference in lh2_reference_select.c and the module, driven the
 * way the swarmit glue drives it, and after every fix, drain and raw-count
 * drain the result, the kept solve bit for bit, the slots and the lines must
 * be identical. Then the rows of the plan's L table, today's outcomes pinned,
 * and the station-rule fixtures PyDotBot's site editor writes
 * (tests/lh2_station_rule.json, a copy of its
 * dotbot/calibration/station_rule.json). `--long` runs ten times the fixes.
 *
 * @copyright Inria, 2026
 */
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lh2_decoder.h"
#include "lh2_geometry.h"
#include "lh2_reference.h"
#include "lh2_reference_select.h"
#include "lh2_select.h"

//=========================== harness ==========================================

static int  _failed = 0;
static int  _passed = 0;
static bool _long   = false;

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

#define MEASURED(...)             \
    do {                          \
        printf("  lh2_select: "); \
        printf(__VA_ARGS__);      \
        printf("\n");             \
    } while (0)

_Static_assert(DB_LH2_SELECT_RECT_LEN == LH2_REF_SELECT_RECT_LEN && DB_LH2_SELECT_LINES_MAX == LH2_REF_SELECT_LINES_MAX, "one layout");
_Static_assert(DB_LH2_STATIONS == LH2_REF_STATIONS, "one station count");

static uint32_t _seed = 20261110U;

/// Uniform in [0, 1), from a 32-bit LCG: reproducible across hosts
static double _uniform(void) {
    _seed = _seed * 1664525U + 1013904223U;
    return (double)(_seed >> 8) / 16777216.0;
}

static double _between(double lo, double hi) {
    return lo + (hi - lo) * _uniform();
}

static uint32_t _below(uint32_t n) {
    return (uint32_t)(_uniform() * n);
}

static bool _same(double a, double b) {
    return memcmp(&a, &b, sizeof(a)) == 0;
}

//=========================== inputs ===========================================

static uint32_t _count(double alpha, uint8_t station) {
    return (uint32_t)llround(alpha / (2.0 * M_PI) * db_lh2_period(station) / DB_LH2_PERIOD_TICKS_PER_COUNT);
}

/// A sweep pair as a station sees it, either order; now and then any two counts
static void _random_counts(uint8_t station, uint32_t *count1, uint32_t *count2) {
    if (_uniform() < 0.05) {
        *count1 = _below(140000);
        *count2 = _below(140000);
        return;
    }
    double   s = _between(M_PI - 0.9, M_PI + 0.9);
    double   d = _between(40.0, 80.0) * M_PI / 180.0;
    uint32_t a = _count(s - d, station), b = _count(s + d, station);
    if (_uniform() < 0.5) {
        *count1 = a;
        *count2 = b;
    } else {
        *count1 = b;
        *count2 = a;
    }
}

/// Mostly well-conditioned, image to floor mm; now and then singular, NaN or infinite
static void _random_homography(float h[3][3]) {
    double k = _between(800, 2000);
    double a = _between(-M_PI, M_PI);
    h[0][0]  = (float)(k * cos(a));
    h[0][1]  = (float)(-k * sin(a) * _between(0.85, 1.0));
    h[0][2]  = (float)_between(500, 3000);
    h[1][0]  = (float)(k * sin(a));
    h[1][1]  = (float)(k * cos(a));
    h[1][2]  = (float)_between(500, 3000);
    h[2][0]  = (float)_between(-0.05, 0.05);
    h[2][1]  = (float)_between(-0.05, 0.05);
    h[2][2]  = 1.0f;
    double u = _uniform();
    if (u < 0.02) {
        h[2][0] = h[2][1] = h[2][2] = 0.0f;
    } else if (u < 0.03) {
        h[_below(3)][_below(3)] = NAN;
    } else if (u < 0.04) {
        h[_below(3)][_below(3)] = INFINITY;
    } else if (u < 0.05) {
        memset(h, 0, 9 * sizeof(float));
    }
}

static uint32_t _clamp_u32(double v) {
    return (!(v > 0)) ? 0 : ((v >= 4294967295.0) ? UINT32_MAX : (uint32_t)v);
}

/// A stored rectangle for a station whose typical solve is (x, y): absent,
/// around it with a margin, tight on it, the site extent, random, or inverted
static void _random_rect(double x, double y, const uint32_t extent[4], uint32_t rect[4]) {
    double u = _uniform();
    if (u < 0.12) {
        rect[0] = rect[1] = rect[2] = rect[3] = UINT32_MAX;
    } else if (u < 0.45 && isfinite(x) && isfinite(y)) {
        double m = _between(0, 400);
        rect[0]  = _clamp_u32(floor(x - m));
        rect[1]  = _clamp_u32(floor(y - m));
        rect[2]  = _clamp_u32(ceil(x + m));
        rect[3]  = _clamp_u32(ceil(y + m));
    } else if (u < 0.6 && isfinite(x) && isfinite(y)) {
        // within a millimetre of the solve, so jitter lands either side
        rect[0] = _clamp_u32(_uniform() < 0.5 ? floor(x) : ceil(x));
        rect[1] = _clamp_u32(floor(y)) - (_uniform() < 0.5 ? 0 : 1);
        rect[2] = rect[0] + _below(3);
        rect[3] = _clamp_u32(ceil(y)) + _below(2);
    } else if (u < 0.8) {
        memcpy(rect, extent, 4 * sizeof(uint32_t));
    } else if (u < 0.97) {
        rect[0] = _below(3000);
        rect[1] = _below(3000);
        rect[2] = rect[0] + _below(3000);
        rect[3] = rect[1] + _below(3000);
    } else {
        rect[0] = 2000 + _below(1000);
        rect[1] = _below(3000);
        rect[2] = _below(2000);
        rect[3] = rect[1] + _below(3000);
    }
}

//=========================== the two sides ====================================

static db_lh2_decoder_t     _dec;
static db_lh2_select_conf_t _conf;
static db_lh2_lines_t       _lines;

/// What swarmit's localization_get_position() does with the module
static bool _new_get_position(db_lh2_fix_t *fix, uint32_t *x, uint32_t *y) {
    db_lh2_select_begin(fix);
    if (_conf.station_mask == 0) {
        return false;
    }
    db_lh2_pair_t pairs[DB_LH2_STATIONS];
    uint8_t       n = db_lh2_select_take_pairs(&_dec, pairs, DB_LH2_STATIONS);
    if (!db_lh2_select_fix(&_conf, pairs, n, fix, &_lines)) {
        return false;
    }
    *x = (uint32_t)fix->x_mm;
    *y = (uint32_t)fix->y_mm;
    return true;
}

/// A sweep decoded on both sides
static void _decode(uint8_t station, uint8_t sweep, uint32_t count) {
    lh2_ref_t *ref                             = lh2_ref_select_slots();
    ref->locations[sweep][station].lfsr_counts = count;
    ref->data_ready[sweep][station]            = LH2_REF_PROCESSED_DATA_AVAILABLE;
    _dec.slot[sweep][station].count            = count;
    _dec.slot[sweep][station].ready            = true;
}

static bool _slots_equal(void) {
    lh2_ref_t *ref = lh2_ref_select_slots();
    for (uint8_t sweep = 0; sweep < 2; sweep++) {
        for (uint8_t station = 0; station < DB_LH2_STATIONS; station++) {
            bool ready = ref->data_ready[sweep][station] == LH2_REF_PROCESSED_DATA_AVAILABLE;
            if (ready != _dec.slot[sweep][station].ready || (ready && ref->locations[sweep][station].lfsr_counts != _dec.slot[sweep][station].count)) {
                return false;
            }
        }
    }
    return true;
}

static void _clear_both(void) {
    lh2_ref_select_clear();
    db_lh2_decoder_init(&_dec);
    memset(&_conf, 0, sizeof(_conf));
    db_lh2_lines_init(&_lines);
}

//=========================== golden comparison ================================

static uint32_t _fixes_compared  = 0;
static uint32_t _fixes_differ    = 0;
static uint32_t _fixes_found     = 0;
static uint32_t _fixes_contested = 0;  ///< fixes where two or more solves landed inside
static uint32_t _drains_compared = 0;
static uint32_t _drains_differ   = 0;
static uint32_t _raws_compared   = 0;
static uint32_t _raws_differ     = 0;
static uint32_t _lines_compared  = 0;
static uint32_t _printed         = 0;

static void _compare_fix(uint32_t session, uint32_t index) {
    lh2_ref_position_t    ref_pos   = { 0 };
    bool                  ref_found = lh2_ref_select_get_position(&ref_pos);
    lh2_ref_select_best_t best      = lh2_ref_select_best();

    db_lh2_fix_t fix;
    uint32_t     x = 0, y = 0;
    bool         found = _new_get_position(&fix, &x, &y);

    // With no station calibrated the reference returns before its fix starts,
    // so its record of the kept solve is the previous fix's
    bool same = ref_found == found && (_conf.station_mask == 0 || best.found == fix.found) && _slots_equal();
    if (same && found) {
        same = best.station == fix.station && _same(best.x, fix.x_mm) && _same(best.y, fix.y_mm) && _same(best.d2, fix.d2) && ref_pos.x == x && ref_pos.y == y;
    }
    _fixes_compared++;
    _fixes_found += found;
    if (!same) {
        _fixes_differ++;
        if (_printed++ < 5) {
            printf("  differ: session %u fix %u, seed %u: reference %d LH%u (%.17g, %.17g), module %d LH%u (%.17g, %.17g)\n", session, index, _seed, ref_found, best.station, best.x, best.y, found, fix.station, fix.x_mm, fix.y_mm);
        }
    }
}

static void _compare_drain(void) {
    uint8_t             max = (uint8_t)_below(11);
    db_lh2_floor_line_t a[10], b[10];
    memset(a, 0, sizeof(a));
    memset(b, 0, sizeof(b));
    uint8_t na = lh2_ref_select_get_lines(a, max);
    uint8_t nb = db_lh2_lines_drain(&_lines, b, max);
    _drains_compared++;
    _lines_compared += na;
    if (na != nb || memcmp(a, b, sizeof(a)) != 0) {
        _drains_differ++;
    }
}

static void _compare_raw(void) {
    uint8_t              max = (uint8_t)_below(17);
    lh2_ref_raw_sample_t a[16];
    db_lh2_pair_t        b[16];
    uint8_t              na = lh2_ref_select_get_raw_counts(a, max);
    uint8_t              nb = 0;
    if (_uniform() < 0.5) {
        nb = db_lh2_select_take_pairs(&_dec, b, max);
    } else {
        // one pair per call, as the swarmit glue drains them
        while (nb < max && db_lh2_select_take_pairs(&_dec, &b[nb], 1) == 1) {
            nb++;
        }
    }
    bool same = na == nb && _slots_equal();
    for (uint8_t i = 0; same && i < na; i++) {
        same = a[i].lh_index == b[i].station && a[i].count1 == b[i].count[0] && a[i].count2 == b[i].count[1];
    }
    _raws_compared++;
    if (!same) {
        _raws_differ++;
    }
}

static void _load(uint32_t mask, float h[16][3][3], uint32_t rects[16][4]) {
    lh2_ref_select_init(h, mask, (const uint32_t(*)[4])rects);
    db_lh2_select_conf_load(&_conf, mask, (const float(*)[3][3])h, (const uint32_t(*)[4])rects);
}

static void test_golden(void) {
    uint32_t fixes   = _long ? 2000000U : 200000U;
    uint32_t session = 0;
    _clear_both();
    while (_fixes_compared < fixes) {
        session++;
        if (_uniform() < 0.7) {
            _clear_both();
        }
        // A calibration: a few stations, some bits past 15, now and then none
        uint32_t mask = 0;
        uint32_t want = (_uniform() < 0.05) ? 0 : 1 + _below(4);
        for (uint32_t i = 0; i < want; i++) {
            mask |= 1U << _below(16);
        }
        if (_uniform() < 0.1) {
            mask |= 1U << (16 + _below(16));
        }
        float    h[16][3][3];
        uint32_t rects[16][4];
        uint32_t extent[4] = { 0, 0, 500 + _below(3000), 500 + _below(3000) };
        uint32_t typical[16][2];
        for (uint8_t s = 0; s < 16; s++) {
            _random_homography(h[s]);
            _random_counts(s, &typical[s][0], &typical[s][1]);
            double xy[2];
            db_lh2_solve(typical[s][0], typical[s][1], s, (const float(*)[3])h[s], xy);
            _random_rect(xy[0], xy[1], extent, rects[s]);
        }
        _load(mask, h, rects);

        double   p_seen = _between(0.05, 0.6);
        uint32_t n      = 20 + _below(40);
        for (uint32_t f = 0; f < n && _fixes_compared < fixes; f++) {
            for (uint8_t s = 0; s < 16; s++) {
                if (_uniform() >= p_seen) {
                    continue;
                }
                uint32_t c1 = typical[s][0], c2 = typical[s][1];
                if (_uniform() < 0.5) {
                    // the robot moved, or the station jitters by a few counts
                    c1 += _below(9) - 4;
                    c2 += _below(9) - 4;
                } else if (_uniform() < 0.2) {
                    _random_counts(s, &c1, &c2);
                }
                if (_uniform() < 0.9) {
                    _decode(s, 0, c1);
                }
                if (_uniform() < 0.9) {
                    _decode(s, 1, c2);
                }
            }
            double u = _uniform();
            if (u < 0.08) {
                _compare_raw();
            } else {
                // how many solves are inside, for the tally of contested fixes
                uint8_t inside = 0;
                for (uint8_t s = 0; s < 16; s++) {
                    if (db_lh2_decoder_pair_ready(&_dec, s) && ((_conf.station_mask >> s) & 1U)) {
                        double xy[2];
                        db_lh2_solve(_dec.slot[0][s].count, _dec.slot[1][s].count, s, (const float(*)[3])_conf.homography[s], xy);
                        inside += isfinite(xy[0]) && isfinite(xy[1]) && db_lh2_select_rect_contains(_conf.rect_mm[s], xy[0], xy[1]);
                    }
                }
                _fixes_contested += inside >= 2;
                _compare_fix(session, f);
            }
            if (_uniform() < 0.4) {
                _compare_drain();
            }
        }
    }
    CHECK(_fixes_differ == 0, "golden: %u of %u fixes differ from the a702520 choice", _fixes_differ, _fixes_compared);
    CHECK(_drains_differ == 0, "golden: %u of %u line drains differ", _drains_differ, _drains_compared);
    CHECK(_raws_differ == 0, "golden: %u of %u raw-count drains differ", _raws_differ, _raws_compared);
    CHECK(_fixes_found > _fixes_compared / 10 && _fixes_contested > _fixes_compared / 50 && _lines_compared > _fixes_compared / 10, "golden: the inputs reach every branch: %u found, %u contested, %u lines", _fixes_found, _fixes_contested, _lines_compared);
    MEASURED("golden comparison: %u fixes (%u found, %u with two or more stations inside), %u line drains (%u lines), %u raw-count drains; %u, %u and %u differ",
             _fixes_compared, _fixes_found, _fixes_contested, _drains_compared, _lines_compared, _raws_compared, _fixes_differ, _drains_differ, _raws_differ);
}

/// The solve-level choice on raw coordinates, edges, NaN and infinities included
static void test_golden_offer_solve(void) {
    static const double specials[] = { 0.0, -0.0, 1.0, 100.0, 999.9999999, 1000.0, 1000.0000001, 4294967295.0, 4294967296.0, -1.0, NAN, INFINITY, -INFINITY };
    uint32_t            differ     = 0;
    uint32_t            n          = 200000;
    for (uint32_t i = 0; i < n; i++) {
        lh2_ref_select_best_t ref = { 0 };
        db_lh2_fix_t          fix;
        db_lh2_select_begin(&fix);
        uint8_t offers = (uint8_t)(1 + _below(5));
        for (uint8_t k = 0; k < offers; k++) {
            uint32_t stored[4], rect[4], rect_ref[4];
            uint32_t extent[4] = { 0, 0, 1000, 1000 };
            double   x         = _between(-200, 1200);
            double   y         = _between(-200, 1200);
            if (_uniform() < 0.2) {
                x = specials[_below(sizeof(specials) / sizeof(specials[0]))];
            }
            if (_uniform() < 0.2) {
                y = specials[_below(sizeof(specials) / sizeof(specials[0]))];
            }
            _random_rect(x, y, extent, stored);
            db_lh2_select_rect_resolve(stored, rect);
            lh2_ref_select_rect_resolve(stored, rect_ref);
            uint8_t station = (uint8_t)_below(16);
            bool    a       = lh2_ref_select_offer_solve(&ref, station, x, y, rect_ref);
            bool    b       = db_lh2_select_offer_solve(&fix, station, x, y, rect);
            if (a != b || memcmp(rect, rect_ref, sizeof(rect)) != 0) {
                differ++;
            }
        }
        if (ref.found != fix.found || (ref.found && (ref.station != fix.station || !_same(ref.x, fix.x_mm) || !_same(ref.y, fix.y_mm) || !_same(ref.d2, fix.d2)))) {
            differ++;
        }
    }
    CHECK(differ == 0, "solve-level choice: %u of %u sequences differ from a702520's lh2_select_offer", differ, n);
}

//=========================== the L rows =======================================

/// A homography that maps the image point to k mm per unit, offset
static void _plain_homography(float h[3][3], float k, float ox, float oy) {
    memset(h, 0, 9 * sizeof(float));
    h[0][0] = k;
    h[0][2] = ox;
    h[1][1] = k;
    h[1][2] = oy;
    h[2][2] = 1.0f;
}

static db_lh2_pair_t _pair(uint8_t station, uint32_t c1, uint32_t c2) {
    db_lh2_pair_t p = { .station = station, .count = { c1, c2 }, .t_us = { 0, 0 } };
    return p;
}

static void test_l1_absent_rectangle(void) {
    uint32_t stored[4] = { UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX }, rect[4];
    db_lh2_select_rect_resolve(stored, rect);
    CHECK(rect[0] == 0 && rect[1] == 0 && rect[2] == 100000 && rect[3] == 100000, "L1: all 0xFF reads as 0 to 100000 mm: %u %u %u %u", rect[0], rect[1], rect[2], rect[3]);
    uint32_t partial[4] = { UINT32_MAX, UINT32_MAX, UINT32_MAX, 5 };
    db_lh2_select_rect_resolve(partial, rect);
    CHECK(rect[0] == UINT32_MAX && rect[3] == 5, "L1: only all four 0xFF is absent");
}

static void test_l2_edges_and_non_finite(void) {
    uint32_t     rect[4] = { 100, 200, 300, 400 };
    db_lh2_fix_t fix;
    db_lh2_select_begin(&fix);
    CHECK(db_lh2_select_rect_contains(rect, 100, 200) && db_lh2_select_rect_contains(rect, 300, 400), "L2: edges are inside");
    CHECK(!db_lh2_select_rect_contains(rect, 300.0000001, 300) && !db_lh2_select_rect_contains(rect, 99.9999999, 300), "L2: just outside is outside");
    CHECK(!db_lh2_select_rect_contains(rect, NAN, 300) && !db_lh2_select_rect_contains(rect, 200, INFINITY) && !db_lh2_select_rect_contains(rect, -INFINITY, 300), "L2: NaN and inf are outside");

    // Through the pair: a solve outside or not finite pushes no line
    db_lh2_select_conf_t conf = { 0 };
    db_lh2_lines_t       lines;
    db_lh2_lines_init(&lines);
    float    h[16][3][3];
    uint32_t rects[16][4];
    _plain_homography(h[0], 1000, 2000, 2000);
    memset(h[1], 0, sizeof(h[1]));
    uint32_t tiny[4] = { 0, 0, 1, 1 };
    memcpy(rects[0], tiny, sizeof(tiny));
    memcpy(rects[1], tiny, sizeof(tiny));
    db_lh2_select_conf_load(&conf, 0x3, (const float(*)[3][3])h, (const uint32_t(*)[4])rects);
    db_lh2_pair_t p0 = _pair(0, 50000, 60000), p1 = _pair(1, 50000, 60000);
    double        xy[2];
    db_lh2_select_begin(&fix);
    CHECK(db_lh2_select_offer_pair(&conf, &fix, &lines, &p0, xy) == DB_LH2_SELECT_OUTSIDE && lines.count == 0, "L2: a finite solve outside its rectangle is dropped, no line");
    CHECK(db_lh2_select_offer_pair(&conf, &fix, &lines, &p1, xy) == DB_LH2_SELECT_NOT_FINITE && lines.count == 0 && !fix.found, "L2: a zero homography solves to NaN, dropped, no line");
    _plain_homography(conf.homography[1], 1000, 2000, 2000);
    conf.homography[1][1][0] = NAN;
    db_lh2_select_offer_pair(&conf, &fix, &lines, &p1, xy);
    CHECK(isfinite(xy[0]) && isnan(xy[1]) && db_lh2_select_offer_pair(&conf, &fix, &lines, &p1, xy) == DB_LH2_SELECT_NOT_FINITE, "L2: a NaN y alone is reported not finite");
}

static void test_l3_nearest_centre(void) {
    uint32_t     a[4] = { 0, 0, 1000, 1000 }, b[4] = { 500, 0, 1500, 1000 };
    db_lh2_fix_t fix;
    db_lh2_select_begin(&fix);
    db_lh2_select_offer_solve(&fix, 0, 800, 500, a);
    db_lh2_select_offer_solve(&fix, 1, 800, 500, b);
    CHECK(fix.found && fix.station == 1, "L3: the one nearer its own centre: LH%u", fix.station);
    db_lh2_select_begin(&fix);
    db_lh2_select_offer_solve(&fix, 3, 500, 500, a);
    db_lh2_select_offer_solve(&fix, 7, 500, 500, a);
    CHECK(fix.found && fix.station == 3, "L3: a tie keeps the one offered first: LH%u", fix.station);
}

static void test_l4_equal_rectangles_flip(void) {
    // The site extent for both: the stations' 2 mm disagreement decides,
    // and it flips with the side of the centre the robot is on (DBK-186's
    // symptom, pinned until the hysteresis lands)
    uint32_t ext[4] = { 0, 0, 1000, 1000 };
    uint32_t flips = 0, last = 99;
    for (int i = 0; i < 200; i++) {
        double       x = 500 + 3.0 * sin(i * 0.3);
        db_lh2_fix_t fix;
        db_lh2_select_begin(&fix);
        db_lh2_select_offer_solve(&fix, 0, x - 1.0, 500, ext);
        db_lh2_select_offer_solve(&fix, 1, x + 1.0, 500, ext);
        if (last != 99 && fix.station != last) {
            flips++;
        }
        last = fix.station;
    }
    CHECK(flips > 10, "L4 (today): equal rectangles flip the station with the robot's side of the centre: %u flips in 200 fixes", flips);
}

static void test_l5_switch_back(void) {
    uint32_t     a[4] = { 0, 0, 1000, 1000 }, b[4] = { 200, 0, 1200, 1000 };
    db_lh2_fix_t fix;
    db_lh2_select_begin(&fix);
    db_lh2_select_offer_solve(&fix, 1, 550, 500, b);
    CHECK(fix.station == 1, "L5: with station 0 out, station 1 is kept");
    db_lh2_select_begin(&fix);
    db_lh2_select_offer_solve(&fix, 0, 550, 500, a);
    db_lh2_select_offer_solve(&fix, 1, 550, 500, b);
    CHECK(fix.station == 0, "L5 (today): station 0 back and nearer its centre, the fix switches back: LH%u", fix.station);
}

static void test_l6_jitter_on_edge(void) {
    db_lh2_select_conf_t conf = { 0 };
    db_lh2_lines_t       lines;
    db_lh2_lines_init(&lines);
    float    h[16][3][3];
    uint32_t rects[16][4];
    _plain_homography(h[0], 1000, 2000, 2000);
    db_lh2_pair_t p = _pair(0, 50000, 60000);
    double        xy[2];
    db_lh2_solve(p.count[0], p.count[1], 0, (const float(*)[3])h[0], xy);
    // The rectangle's x_max on the solve: a count either way moves it across
    rects[0][0] = 0;
    rects[0][1] = 0;
    rects[0][2] = (uint32_t)floor(xy[0]);
    rects[0][3] = 100000;
    db_lh2_select_conf_load(&conf, 0x1, (const float(*)[3][3])h, (const uint32_t(*)[4])rects);
    uint32_t in = 0, out = 0;
    for (int i = 0; i < 400; i++) {
        db_lh2_pair_t q = _pair(0, p.count[0] + _below(61) - 30, p.count[1] + _below(61) - 30);
        db_lh2_fix_t  fix;
        db_lh2_select_begin(&fix);
        db_lh2_floor_line_t drained[DB_LH2_SELECT_LINES_MAX];
        db_lh2_lines_drain(&lines, drained, DB_LH2_SELECT_LINES_MAX);
        db_lh2_select_offer_pair(&conf, &fix, &lines, &q, xy);
        if (lines.count) {
            in++;
        } else {
            out++;
        }
    }
    CHECK(in > 20 && out > 20, "L6 (today): a solve jittering on an edge puts its lines in and out: in %u, out %u of 400", in, out);
}

static void test_l7_uncalibrated_taken_not_solved(void) {
    db_lh2_decoder_t dec;
    db_lh2_decoder_init(&dec);
    for (uint8_t sweep = 0; sweep < 2; sweep++) {
        dec.slot[sweep][5].count = 1000 + sweep;
        dec.slot[sweep][5].ready = true;
    }
    db_lh2_select_conf_t conf = { 0 };
    conf.station_mask         = 0x1;
    db_lh2_lines_t lines;
    db_lh2_lines_init(&lines);
    db_lh2_pair_t pairs[16];
    uint8_t       n = db_lh2_select_take_pairs(&dec, pairs, 16);
    db_lh2_fix_t  fix;
    double        xy[2];
    db_lh2_select_begin(&fix);
    CHECK(n == 1 && !db_lh2_decoder_pair_ready(&dec, 5), "L7: an uncalibrated station's pair is taken, so never stale");
    CHECK(db_lh2_select_offer_pair(&conf, &fix, &lines, &pairs[0], xy) == DB_LH2_SELECT_UNCALIBRATED && isnan(xy[0]) && !fix.found && lines.count == 0, "L7: but not solved");
}

static void test_l9_singular_homography(void) {
    db_lh2_select_conf_t conf = { 0 };
    db_lh2_lines_t       lines;
    db_lh2_lines_init(&lines);
    float    h[16][3][3];
    uint32_t rects[16][4];
    _plain_homography(h[0], 1000, 2000, 2000);
    h[0][2][2] = 0;  // the point at infinity
    _plain_homography(h[1], 1000, 2000, 2000);
    h[1][0][0] = NAN;
    for (uint8_t s = 0; s < 2; s++) {
        rects[s][0] = rects[s][1] = rects[s][2] = rects[s][3] = UINT32_MAX;
    }
    db_lh2_select_conf_load(&conf, 0x3, (const float(*)[3][3])h, (const uint32_t(*)[4])rects);
    db_lh2_pair_t pairs[2] = { _pair(0, 50000, 60000), _pair(1, 50000, 60000) };
    db_lh2_fix_t  fix;
    CHECK(!db_lh2_select_fix(&conf, pairs, 2, &fix, &lines) && lines.count == 0, "L9: a singular or NaN homography gives no solve and no line");
}

static void test_l10_ring(void) {
    db_lh2_lines_t lines;
    db_lh2_lines_init(&lines);
    for (uint8_t i = 0; i < 5; i++) {
        db_lh2_floor_line_t pair[2] = { { .station = i, .sweep = 0 }, { .station = i, .sweep = 1 } };
        db_lh2_lines_push(&lines, pair);
    }
    db_lh2_floor_line_t out[10];
    uint8_t             n  = db_lh2_lines_drain(&lines, out, 3);
    bool                ok = n == 3 && out[0].station == 1 && out[0].sweep == 0 && out[1].station == 1 && out[2].station == 2 && out[2].sweep == 0;
    n                      = db_lh2_lines_drain(&lines, out, 10);
    ok                     = ok && n == 5 && out[0].station == 2 && out[0].sweep == 1 && out[4].station == 4 && out[4].sweep == 1;
    CHECK(ok, "L10: ten lines into eight: the oldest pair overwritten, drained oldest first");
    CHECK(db_lh2_lines_drain(&lines, out, 10) == 0, "L10: drained empty");
}

static void test_l11_raw_drain(void) {
    db_lh2_decoder_t dec;
    db_lh2_decoder_init(&dec);
    static const uint8_t ready[] = { 0, 3, 9, 15 };
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t sweep = 0; sweep < 2; sweep++) {
            dec.slot[sweep][ready[i]].count = 100U * ready[i] + sweep;
            dec.slot[sweep][ready[i]].ready = true;
        }
    }
    dec.slot[0][7].ready = true;  // one sweep only: not a pair
    db_lh2_pair_t out[16];
    uint8_t       n  = db_lh2_select_take_pairs(&dec, out, 3);
    bool          ok = n == 3 && out[0].station == 0 && out[1].station == 3 && out[2].station == 9 && out[2].count[0] == 900 && out[2].count[1] == 901;
    CHECK(ok && db_lh2_decoder_pair_ready(&dec, 15), "L11: every ready station in order, calibrated or not; past max stays ready");
    n = db_lh2_select_take_pairs(&dec, out, 16);
    CHECK(n == 1 && out[0].station == 15 && dec.slot[0][7].ready, "L11: the rest, and a lone sweep stays");
}

static void test_conf_load(void) {
    db_lh2_select_conf_t conf = { 0 };
    float                h[16][3][3];
    uint32_t             rects[16][4];
    for (uint8_t s = 0; s < 16; s++) {
        _plain_homography(h[s], 1000 + s, 0, 0);
        rects[s][0] = rects[s][1] = rects[s][2] = rects[s][3] = s;
    }
    db_lh2_select_conf_load(&conf, 0x10005, (const float(*)[3][3])h, (const uint32_t(*)[4])rects);
    CHECK(conf.station_mask == 0x5 && conf.homography[2][0][0] == 1002.0f && conf.rect_mm[2][3] == 2 && conf.homography[1][0][0] == 0.0f, "conf: the mask's stations loaded, bits past 15 dropped");
    db_lh2_select_conf_load(&conf, 0x2, (const float(*)[3][3])h, (const uint32_t(*)[4])rects);
    CHECK(conf.station_mask == 0x2 && conf.homography[2][0][0] == 1002.0f, "conf: a reload replaces the mask and keeps the other stations' data");
}

//=========================== the station-rule fixtures ========================

/// A JSON value, enough of it for the fixture file
typedef enum { J_NULL,
               J_NUM,
               J_STR,
               J_ARR,
               J_OBJ } jtype_t;

typedef struct jval {
    jtype_t      type;
    double       num;
    char        *str;   ///< J_STR
    char        *key;   ///< the member's key, inside a J_OBJ
    struct jval *kids;  ///< J_ARR and J_OBJ
    uint32_t     n;
} jval_t;

static const char *_skip(const char *p) {
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    return p;
}

static const char *_parse(const char *p, jval_t *v);

static const char *_parse_str(const char *p, char **out) {
    const char *start = ++p;
    while (*p && *p != '"') {
        p += (*p == '\\' && p[1]) ? 2 : 1;
    }
    size_t len = (size_t)(p - start);
    *out       = malloc(len + 1);
    memcpy(*out, start, len);
    (*out)[len] = 0;
    return *p ? p + 1 : p;
}

static const char *_parse_list(const char *p, jval_t *v, char close, bool keys) {
    p       = _skip(p + 1);
    v->kids = NULL;
    v->n    = 0;
    while (*p && *p != close) {
        v->kids     = realloc(v->kids, (v->n + 1) * sizeof(jval_t));
        jval_t *kid = &v->kids[v->n++];
        kid->key    = NULL;
        if (keys) {
            p = _skip(_parse_str(p, &kid->key));
            p = _skip(p + 1);  // ':'
        }
        p = _skip(_parse(p, kid));
        if (*p == ',') {
            p = _skip(p + 1);
        }
    }
    return *p ? p + 1 : p;
}

static const char *_parse(const char *p, jval_t *v) {
    p = _skip(p);
    if (*p == '{') {
        v->type = J_OBJ;
        return _parse_list(p, v, '}', true);
    }
    if (*p == '[') {
        v->type = J_ARR;
        return _parse_list(p, v, ']', false);
    }
    if (*p == '"') {
        v->type = J_STR;
        return _parse_str(p, &v->str);
    }
    if (strncmp(p, "null", 4) == 0) {
        v->type = J_NULL;
        return p + 4;
    }
    char *end;
    v->type = J_NUM;
    v->num  = strtod(p, &end);
    return end;
}

static void _free(jval_t *v) {
    for (uint32_t i = 0; (v->type == J_ARR || v->type == J_OBJ) && i < v->n; i++) {
        _free(&v->kids[i]);
        free(v->kids[i].key);
    }
    if (v->type == J_ARR || v->type == J_OBJ) {
        free(v->kids);
    }
    if (v->type == J_STR) {
        free(v->str);
    }
}

static const jval_t *_member(const jval_t *obj, const char *key) {
    for (uint32_t i = 0; obj && obj->type == J_OBJ && i < obj->n; i++) {
        if (obj->kids[i].key && strcmp(obj->kids[i].key, key) == 0) {
            return &obj->kids[i];
        }
    }
    return NULL;
}

static double _number(const jval_t *v) {
    return (v->type == J_STR) ? strtod(v->str, NULL) : v->num;
}

static void test_station_rule_fixtures(const char *path) {
    FILE *f = fopen(path, "rb");
    CHECK(f != NULL, "station rule: %s opens", path);
    if (!f) {
        return;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *text                            = malloc((size_t)size + 1);
    text[fread(text, 1, (size_t)size, f)] = 0;
    fclose(f);

    jval_t root;
    _parse(text, &root);
    const jval_t *cases = _member(&root, "cases");
    CHECK(cases && cases->type == J_ARR && cases->n > 0, "station rule: the file has cases");
    uint32_t agree = 0, total = cases ? cases->n : 0;
    for (uint32_t i = 0; i < total; i++) {
        const jval_t         *c      = &cases->kids[i];
        const jval_t         *rects  = _member(c, "rects");
        const jval_t         *solves = _member(c, "solves");
        const jval_t         *expect = _member(c, "expect");
        const jval_t         *name   = _member(c, "name");
        db_lh2_fix_t          fix;
        lh2_ref_select_best_t ref = { 0 };
        db_lh2_select_begin(&fix);
        for (uint32_t k = 0; solves && k < solves->n; k++) {
            const jval_t *s       = &solves->kids[k];
            uint8_t       station = (uint8_t)s->kids[0].num;
            double        x = _number(&s->kids[1]), y = _number(&s->kids[2]);
            char          key[4];
            snprintf(key, sizeof(key), "%u", station);
            const jval_t *r         = _member(rects, key);
            uint32_t      stored[4] = { 0 }, rect[4], rect_ref[4];
            for (uint8_t j = 0; r && j < 4; j++) {
                stored[j] = (uint32_t)r->kids[j].num;
            }
            db_lh2_select_rect_resolve(stored, rect);
            lh2_ref_select_rect_resolve(stored, rect_ref);
            // The glue drops a non-finite solve before offering it
            if (isfinite(x) && isfinite(y)) {
                db_lh2_select_offer_solve(&fix, station, x, y, rect);
                lh2_ref_select_offer_solve(&ref, station, x, y, rect_ref);
            }
        }
        bool want_none = expect->type == J_NULL;
        bool ok        = want_none ? (!fix.found && !ref.found) : (fix.found && ref.found && fix.station == (uint8_t)expect->num && ref.station == fix.station);
        CHECK(ok, "station rule \"%s\": expected %s%.0f, module %s%u, a702520 %s%u", name ? name->str : "?", want_none ? "none" : "LH", want_none ? 0.0 : expect->num, fix.found ? "LH" : "none ", fix.station, ref.found ? "LH" : "none ", ref.station);
        agree += ok;
    }
    MEASURED("station rule fixtures: %u of %u cases agree in the module and in the a702520 copy", agree, total);
    _free(&root);
    free(text);
}

int main(int argc, char **argv) {
    const char *fixtures = "tests/lh2_station_rule.json";
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--long") == 0) {
            _long = true;
        } else {
            fixtures = argv[i];
        }
    }
    test_l1_absent_rectangle();
    test_l2_edges_and_non_finite();
    test_l3_nearest_centre();
    test_l4_equal_rectangles_flip();
    test_l5_switch_back();
    test_l6_jitter_on_edge();
    test_l7_uncalibrated_taken_not_solved();
    test_l9_singular_homography();
    test_l10_ring();
    test_l11_raw_drain();
    test_conf_load();
    test_station_rule_fixtures(fixtures);
    test_golden_offer_solve();
    test_golden();
    printf("lh2_select%s: %d passed, %d failed\n", _long ? " (long)" : "", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
