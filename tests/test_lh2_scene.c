/**
 * @file
 * @brief  Host test of the LH2 path end to end: generated captures to the estimator
 *
 * A scene puts two stations over a robot driven by the control core's plant.
 * Every 10 ms tick, the sweeps that crossed the photodiode become captures,
 * which go through the capture queue into drv/lh2_decoder; when the control
 * core asks for a fix, the pairs go through drv/lh2_select the way the
 * swarmit glue drives it, and the fix and the floor lines go into
 * db_control_tick(). Rows E1-E6 and L8 of the plan's matrix, today's outcomes
 * pinned; E4 pins the late duplicate's bimodal line, which the decoder fixes
 * will change on purpose.
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dotbot_control.h"
#include "geometry.h"
#include "lh2_decoder.h"
#include "lh2_geometry.h"
#include "lh2_scene.h"
#include "lh2_select.h"
#include "lh2_synth.h"
#include "plant.h"

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

#define MEASURED(...)            \
    do {                         \
        printf("  lh2_scene: "); \
        printf(__VA_ARGS__);     \
        printf("\n");            \
    } while (0)

#define TICK_US    (10000.0)
#define HISTORY    (64U)  ///< ticks of photodiode truth kept, for truth at a capture's time
#define EVENTS_MAX (512U)
#define STATION_A  (0U)
#define STATION_B  (1U)

//=========================== the scene ========================================

typedef struct {
    plant_t              plant;
    db_control_t         control;
    db_control_output_t  out;
    lh2_scene_station_t  st[4];
    uint8_t              n;
    db_lh2_decoder_t     dec;
    db_lh2_queue_t       queue;
    db_lh2_select_conf_t conf;
    db_lh2_lines_t       lines;
    lh2_synth_rng_t      rng;
    uint64_t             base_us;
    double               t_us;
    uint32_t             fix_sequence;
    uint32_t             fix_x;
    uint32_t             fix_y;
    bool                 open_loop;  ///< the plant runs on pwm_left and pwm_right, not on the control core
    int8_t               pwm_left;
    int8_t               pwm_right;
    // the last read, for the rows to inspect
    bool          read;        ///< this tick read a fix
    db_lh2_fix_t  fix;         ///< the fix that read built
    uint8_t       pairs;       ///< pairs taken
    db_lh2_pair_t pair[4];     ///< and which
    uint8_t       line_count;  ///< lines drained
    double        truth[2];    ///< the photodiode at the end of the tick
    double        hist_t[HISTORY];
    double        hist_x[HISTORY];
    double        hist_y[HISTORY];
    uint32_t      ticks;
} scene_t;

static scene_t           _s;
static lh2_synth_event_t _events[EVENTS_MAX];

/// The photodiode at scene time t_us, from the last HISTORY ticks
static void _truth_at(const scene_t *s, double t_us, double xy[2]) {
    uint32_t newest = (s->ticks + HISTORY - 1) % HISTORY;
    xy[0]           = s->hist_x[newest];
    xy[1]           = s->hist_y[newest];
    for (uint32_t k = 1; k < HISTORY && k < s->ticks; k++) {
        uint32_t i = (s->ticks + HISTORY - 1 - k) % HISTORY;
        uint32_t j = (i + 1) % HISTORY;
        if (s->hist_t[i] <= t_us && t_us <= s->hist_t[j]) {
            double f = (t_us - s->hist_t[i]) / (s->hist_t[j] - s->hist_t[i]);
            xy[0]    = s->hist_x[i] + f * (s->hist_x[j] - s->hist_x[i]);
            xy[1]    = s->hist_y[i] + f * (s->hist_y[j] - s->hist_y[i]);
            return;
        }
    }
}

static void _record(scene_t *s) {
    uint32_t i   = s->ticks % HISTORY;
    s->hist_t[i] = s->t_us;
    s->hist_x[i] = s->plant.px[0];
    s->hist_y[i] = s->plant.py[0];
    s->ticks++;
}

/// Two stations over a 2 m floor, the robot at (x, y), heading 0, at rest
static void _scene_init(scene_t *s, float x, float y, uint64_t base_us, uint32_t seed) {
    memset(s, 0, sizeof(*s));
    plant_init(&s->plant, x, y, 0.0f, 0.0f, seed);
    db_control_init(&s->control, &db_control_default_conf);
    s->rng.state = seed;
    s->base_us   = base_us;
    s->n         = 2;
    lh2_scene_station_init(&s->st[0], STATION_A, 900, 1100, 1600, 0.3, 0.02, -0.01, 1234.5);
    lh2_scene_station_init(&s->st[1], STATION_B, 1150, 950, 1500, -2.2, -0.015, 0.025, 7777.7);
    float    h[DB_LH2_SELECT_STATIONS][3][3];
    uint32_t rects[DB_LH2_SELECT_STATIONS][DB_LH2_SELECT_RECT_LEN];
    memset(h, 0, sizeof(h));
    memset(rects, 0xFF, sizeof(rects));
    for (uint8_t i = 0; i < s->n; i++) {
        memcpy(h[s->st[i].station], s->st[i].h, sizeof(h[0]));
    }
    db_lh2_select_conf_load(&s->conf, (1U << STATION_A) | (1U << STATION_B), (const float(*)[3][3])h, (const uint32_t(*)[DB_LH2_SELECT_RECT_LEN])rects);
    db_lh2_decoder_init(&s->dec);
    db_lh2_queue_clear(&s->queue);
    db_lh2_lines_init(&s->lines);
    _record(s);
}

/// What swarmit_keep_alive() and the app's position read do
static void _read_fix(scene_t *s, db_control_input_t *in) {
    s->read  = true;
    s->pairs = 0;
    db_lh2_select_begin(&s->fix);
    for (uint8_t station = 0; station < DB_LH2_STATIONS; station++) {
        db_lh2_pair_t pair;
        if (!db_lh2_decoder_take_pair(&s->dec, station, &pair)) {
            continue;
        }
        if (s->pairs < 4) {
            s->pair[s->pairs++] = pair;
        }
        double xy[2];
        db_lh2_select_offer_pair(&s->conf, &s->fix, &s->lines, &pair, xy);
    }
    if (s->fix.found) {
        s->fix_sequence++;
        s->fix_x = (uint32_t)s->fix.x_mm;
        s->fix_y = (uint32_t)s->fix.y_mm;
    }
    in->fix_sequence = s->fix_sequence;
    in->fix_x        = s->fix_x;
    in->fix_y        = s->fix_y;
    db_lh2_floor_line_t lines[DB_CONTROL_LINES_MAX];
    s->line_count = db_lh2_lines_drain(&s->lines, lines, DB_CONTROL_LINES_MAX);
    db_control_lines(&s->control, lines, s->line_count);
}

static void _tick(scene_t *s) {
    double from[2] = { s->plant.px[0], s->plant.py[0] };
    if (s->open_loop) {
        s->plant.pwm_left    = s->pwm_left;
        s->plant.pwm_right   = s->pwm_right;
        s->plant.brake_left  = false;
        s->plant.brake_right = false;
    }
    db_control_input_t in;
    plant_step(&s->plant, &in);
    double to[2] = { s->plant.px[0], s->plant.py[0] };
    size_t n     = lh2_scene_window(s->st, s->n, s->t_us, s->t_us + TICK_US, from, to, s->base_us, &s->rng, _events, EVENTS_MAX);
    for (size_t i = 0; i < n; i++) {
        // The SPIM handler queues the capture, then decodes one
        db_lh2_queue_push(&s->queue, _events[i].spi, _events[i].t_us);
        db_lh2_capture_t cap;
        if (db_lh2_queue_pop(&s->queue, &cap)) {
            db_lh2_decoder_feed(&s->dec, &cap, _events[i].t_us);
        }
    }
    s->t_us += TICK_US;
    s->truth[0] = to[0];
    s->truth[1] = to[1];
    _record(s);

    in.fix_sequence  = s->fix_sequence;
    in.fix_x         = s->fix_x;
    in.fix_y         = s->fix_y;
    in.elapsed_ticks = 1;
    s->read          = false;
    if (db_control_fix_due(&s->control, 1)) {
        _read_fix(s, &in);
    }
    db_control_tick(&s->control, &in, &s->out);
    if (!s->open_loop) {
        plant_apply(&s->plant, &s->out);
    }
}

static void _run(scene_t *s, uint32_t ticks) {
    for (uint32_t i = 0; i < ticks; i++) {
        _tick(s);
    }
}

static int _cmp_double(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static double _percentile(double *v, uint32_t n, double p) {
    if (n == 0) {
        return NAN;
    }
    qsort(v, n, sizeof(v[0]), _cmp_double);
    uint32_t i = (uint32_t)(p * (n - 1) + 0.5);
    return v[i];
}

//=========================== the generator ====================================

static void test_scene_round_trip(void) {
    lh2_scene_station_t st;
    lh2_scene_station_init(&st, 3, 1000, 1000, 1500, 0.7, 0.01, 0.02, 0);
    uint32_t seed = 99, worst_count = 0, failed = 0;
    double   worst_mm = 0;
    for (int i = 0; i < 20000; i++) {
        seed     = seed * 1664525U + 1013904223U;
        double x = 200 + (seed >> 8) % 1600;
        seed     = seed * 1664525U + 1013904223U;
        double y = 200 + (seed >> 8) % 1600;
        double cam[2], counts[2];
        lh2_scene_camera_point(&st, x, y, cam);
        if (!lh2_scene_counts(st.station, cam, counts)) {
            failed++;
            continue;
        }
        uint32_t c1 = (uint32_t)llround(counts[0]), c2 = (uint32_t)llround(counts[1]);
        double   back[2], again[2];
        db_lh2_camera_point(c1, c2, st.station, back);
        lh2_scene_counts(st.station, back, again);
        uint32_t d  = (uint32_t)fmax(fabs(again[0] - c1), fabs(again[1] - c2));
        worst_count = d > worst_count ? d : worst_count;
        double xy[2];
        db_lh2_solve(c1, c2, st.station, (const float(*)[3])st.h, xy);
        double e = hypot(xy[0] - x, xy[1] - y);
        worst_mm = e > worst_mm ? e : worst_mm;
    }
    CHECK(failed == 0 && worst_count == 0, "scene: the camera inversion round-trips through db_lh2_camera_point() within a count: %u off, %u points unseen", worst_count, failed);
    CHECK(worst_mm < 0.5, "scene: rounded counts solve back to the floor point within 0.5 mm: %.3f mm", worst_mm);
    MEASURED("camera inversion: 20000 floor points, rounded counts solve back within %.3f mm", worst_mm);
}

//=========================== the E rows =======================================

static void test_e1_at_rest(void) {
    scene_t *s = &_s;
    _scene_init(s, 1000, 1000, 0, 11);
    _run(s, 50);
    uint32_t reads = 0, four = 0, found = 0;
    double   worst = 0;
    for (int i = 0; i < 600; i++) {
        _tick(s);
        if (!s->read) {
            continue;
        }
        reads++;
        four += s->line_count == 4;
        if (s->fix.found) {
            found++;
            double e = hypot(s->fix.x_mm - s->truth[0], s->fix.y_mm - s->truth[1]);
            worst    = e > worst ? e : worst;
        }
    }
    CHECK(reads >= 55 && found == reads, "E1: a fix at every read, %u of %u", found, reads);
    CHECK(four == reads, "E1: four lines per fix, %u of %u reads", four, reads);
    CHECK(worst < 0.5, "E1: every fix within 0.5 mm of the truth: worst %.3f mm", worst);
    MEASURED("E1 at rest, two stations: %u fixes, worst %.3f mm from the truth, four lines in %u", found, worst, four);
}

/// Fix errors against the photodiode at the pair's later sweep, on an open-loop drive,
/// split by whether the pair's sweeps came in revolution order or straddled two
typedef struct {
    double   same[4000];
    double   mixed[4000];
    uint32_t n_same;
    uint32_t n_mixed;
    double   est_max;  ///< estimator to truth past the first quarter, mm
} drive_t;

static void _drive(scene_t *s, int8_t pwm_left, int8_t pwm_right, uint32_t ticks, drive_t *d) {
    s->open_loop = true;
    s->pwm_left  = pwm_left;
    s->pwm_right = pwm_right;
    for (uint32_t i = 0; i < ticks; i++) {
        _tick(s);
        double e = hypot(s->control.estimator.x - s->plant.x, s->control.estimator.y - s->plant.y);
        if (i > ticks / 4 && e > d->est_max) {
            d->est_max = e;
        }
        if (!s->read || !s->fix.found) {
            continue;
        }
        for (uint8_t k = 0; k < s->pairs; k++) {
            if (s->pair[k].station != s->fix.station) {
                continue;
            }
            // Slot 0 holds the sweep that first came after a cold start, slot 1 the other;
            // a pair is mixed when its two sweeps are further apart than one sweep gap
            uint32_t a = s->pair[k].t_us[0], b = s->pair[k].t_us[1];
            int32_t  dt    = (int32_t)(b - a);
            uint32_t later = (dt > 0) ? b : a;
            double   t     = (double)(uint32_t)(later - (uint32_t)s->base_us) - LH2_SYNTH_CAPTURE_US;
            double   truth[2];
            _truth_at(s, t, truth);
            double err = hypot(s->fix.x_mm - truth[0], s->fix.y_mm - truth[1]);
            if (dt < 0) {
                d->mixed[d->n_mixed++ % 4000] = err;
            } else {
                d->same[d->n_same++ % 4000] = err;
            }
        }
    }
}

static void test_e2_drive_and_l8_speed(void) {
    static drive_t      d;
    static const int8_t pwm[] = { 42, 52, 61, 71, 81 };  // about 100, 200, 300, 400, 500 mm/s
    double              p95_mixed[5], p95_same[5];
    for (uint8_t k = 0; k < 5; k++) {
        memset(&d, 0, sizeof(d));
        double v = 0;
        // Eight rotor phases against the 100 ms read, so reads land in every part of a revolution
        for (uint8_t j = 0; j < 8; j++) {
            scene_t *s = &_s;
            _scene_init(s, 400, 300, 0, 21 + k * 8 + j);
            for (uint8_t i = 0; i < s->n; i++) {
                s->st[i].t0_us += j * lh2_synth_period_us(s->st[i].station) / 8.0;
            }
            db_control_seed(&s->control, 400, 300, 0);
            _run(s, 30);
            _drive(s, pwm[k], pwm[k], 150, &d);
            v = 0.5 * (s->plant.v_left + s->plant.v_right);
        }
        p95_same[k]  = _percentile(d.same, d.n_same, 0.95);
        p95_mixed[k] = _percentile(d.mixed, d.n_mixed, 0.95);
        MEASURED("L8 straight at %.0f mm/s: %u fixes from pairs in revolution order, error to the truth at the later sweep p95 %.2f mm; %u from pairs straddling two revolutions, p95 %.2f mm; estimator within %.1f mm",
                 v, d.n_same, p95_same[k], d.n_mixed, p95_mixed[k], d.est_max);
        if (k == 2) {
            CHECK(d.n_same + d.n_mixed >= 100 && d.est_max < 15.0, "E2: straight at 300 mm/s, the estimator tracks within 15 mm: %.1f mm", d.est_max);
            CHECK(d.n_mixed * 10 > d.n_same, "L8 (today): pairs straddling two revolutions are solved: %u, against %u in order", d.n_mixed, d.n_same);
        }
    }
    CHECK(p95_mixed[4] > 2 * p95_mixed[0] && p95_mixed[4] > p95_same[4], "L8 (today): a straddling pair's error grows with speed: p95 %.2f mm at 100 mm/s, %.2f mm at 500 mm/s, %.2f mm for pairs in order at 500", p95_mixed[0], p95_mixed[4], p95_same[4]);

    memset(&d, 0, sizeof(d));
    scene_t *s = &_s;
    _scene_init(s, 1000, 1000, 0, 31);
    db_control_seed(&s->control, 1000, 1000, 0);
    _run(s, 30);
    _drive(s, 60, -60, 300, &d);
    MEASURED("E2 spin in place: %u fixes, error p95 %.2f mm in order, %.2f mm straddling; estimator within %.1f mm", d.n_same + d.n_mixed, _percentile(d.same, d.n_same, 0.95), _percentile(d.mixed, d.n_mixed, 0.95), d.est_max);
    CHECK(d.n_same + d.n_mixed >= 25 && d.est_max < 15.0, "E2: spinning in place, the estimator tracks within 15 mm: %.1f mm", d.est_max);
}

static void test_e3_station_dark(void) {
    scene_t *s = &_s;
    _scene_init(s, 1000, 1000, 0, 41);
    db_control_seed(&s->control, 1000, 1000, 0);
    s->st[1].silent_us[0] = 5e6;
    s->st[1].silent_us[1] = 20e6;
    _run(s, 500);
    uint32_t dark_reads = 0, dark_single = 0;
    for (int i = 0; i < 1500; i++) {
        _tick(s);
        if (s->read && s->t_us > 5.2e6) {
            dark_reads++;
            dark_single += s->fix.found && s->fix.station == STATION_A && s->line_count == 2;
        }
    }
    // back at 20 s: B's lines are left out while it settles, then fused
    uint32_t settling = 0, settled = 0, first_accepted_tick = 0;
    for (int i = 0; i < 500; i++) {
        _tick(s);
        const db_control_fusion_record_t *r = &s->control.last_fusion;
        if (!s->read) {
            continue;
        }
        for (uint8_t k = 0; k < r->count; k++) {
            if ((r->station_sweep[k] >> 1) != STATION_B) {
                continue;
            }
            if (r->result[k] == DB_POSE_ESTIMATOR_ACCEPTED) {
                settled++;
                first_accepted_tick = first_accepted_tick ? first_accepted_tick : (uint32_t)i;
            } else if (!settled) {
                settling++;
            }
        }
    }
    CHECK(dark_reads > 100 && dark_single == dark_reads, "E3: with B dark, every fix is A's alone, two lines: %u of %u", dark_single, dark_reads);
    CHECK(first_accepted_tick >= 240 && first_accepted_tick <= 280 && settled > 30, "E3: B's lines left out for 2.5 s on return, then fused: first accepted %u ticks after, %u lines left out before", first_accepted_tick, settling);
    MEASURED("E3 station B dark 15 s: %u single-station fixes; on return its lines left out %u times, first fused after %u ticks", dark_single, settling, first_accepted_tick);
}

static void test_e4_late_duplicate(void) {
    scene_t *s = &_s;
    _scene_init(s, 1000, 1000, 0, 51);
    db_control_seed(&s->control, 1000, 1000, 0);
    s->st[1].dup_share       = 0.3;
    s->st[1].dup_delay_us[0] = 30;
    s->st[1].dup_delay_us[1] = 36;
    s->st[1].dup_sweeps      = 1U << 0;
    _run(s, 100);
    static double innov[1000], far[1000];
    uint32_t      n = 0, near = 0, nfar = 0;
    for (int i = 0; i < 6000 && n < 1000; i++) {
        _tick(s);
        const db_control_fusion_record_t *r = &s->control.last_fusion;
        if (!s->read) {
            continue;
        }
        for (uint8_t k = 0; k < r->count; k++) {
            if (r->station_sweep[k] == ((STATION_B << 1) | 0U) && !isnan(r->innovation_mm[k])) {
                innov[n++] = r->innovation_mm[k];
            }
        }
    }
    for (uint32_t i = 0; i < n; i++) {
        if (fabs(innov[i]) < 3.0) {
            near++;
        } else {
            far[nfar++] = innov[i];
        }
    }
    double   mode    = _percentile(far, nfar, 0.5);
    uint32_t in_mode = 0;
    for (uint32_t i = 0; i < nfar; i++) {
        in_mode += fabs(far[i] - mode) < 5.0;
    }
    CHECK(n >= 500, "E4: station B sweep 0's line read %u times", n);
    CHECK(fabs(mode) > 10.0 && fabs(mode) < 40.0 && in_mode * 10 >= nfar * 9, "E4 (today): a second mode %.1f mm off, holding %u of %u off-zero reads", mode, in_mode, nfar);
    CHECK(nfar * 100 >= n * 15 && nfar * 100 <= n * 45, "E4 (today): the late duplicate is read in %u of %u reads, about the injected 30 %%", nfar, n);
    MEASURED("E4 late duplicate on B's sweep 0 in 30 %% of passes, 30-36 us late: %u reads, %u within 3 mm of the line, %u in a mode at %+.1f mm", n, near, in_mode, mode);
}

static void test_e5_moved(void) {
    scene_t *s = &_s;
    _scene_init(s, 800, 800, 0, 61);
    db_control_seed(&s->control, 800, 800, 0);
    _run(s, 300);
    uint32_t kidnaps = s->control.estimator.kidnaps;
    // picked up and put down 500 mm away between two fixes
    s->plant.x += 500.0f;
    for (unsigned i = 0; i <= PLANT_FIX_AGE; i++) {
        s->plant.px[i] += 500.0f;
    }
    _run(s, 300);
    CHECK(s->control.estimator.kidnaps == kidnaps + 1 && s->control.estimator.status == DB_POSE_ESTIMATOR_SEEDING, "E5: moved 500 mm at rest, the estimator sees a kidnap and seeds again: %u kidnaps, status %u", s->control.estimator.kidnaps - kidnaps, s->control.estimator.status);
    double e = hypot((double)s->fix_x - s->truth[0], (double)s->fix_y - s->truth[1]);
    CHECK(e < 1.5, "E5: the fixes follow the robot: %.2f mm", e);
    static drive_t d;
    _drive(s, 55, 55, 300, &d);
    double est = hypot(s->control.estimator.x - s->plant.x, s->control.estimator.y - s->plant.y);
    CHECK(s->control.estimator.status == DB_POSE_ESTIMATOR_TRACKING && est < 15.0, "E5: driving off, it reseeds and tracks: status %u, %.1f mm", s->control.estimator.status, est);
    MEASURED("E5 moved 500 mm: one kidnap, seeding; after 3 s of driving status %u, estimator %.1f mm from the truth", s->control.estimator.status, est);
}

static void test_e6_wrap(void) {
    // The same drive twice, once with the 2^32 us wrap 2 s in: identical fixes
    static uint32_t seq[2][400], fx[2][400], fy[2][400];
    uint32_t        reads[2] = { 0, 0 };
    uint64_t        base[2]  = { 0, 4294967296ULL - 2000000ULL };
    for (uint8_t r = 0; r < 2; r++) {
        scene_t *s = &_s;
        _scene_init(s, 700, 300, base[r], 71);
        db_control_seed(&s->control, 700, 300, 0);
        s->open_loop = true;
        s->pwm_left  = 61;
        s->pwm_right = 61;
        for (int i = 0; i < 400; i++) {
            _tick(s);
            if (s->read && reads[r] < 400) {
                seq[r][reads[r]] = s->fix_sequence;
                fx[r][reads[r]]  = s->fix_x;
                fy[r][reads[r]]  = s->fix_y;
                reads[r]++;
            }
        }
    }
    bool     same = reads[0] == reads[1] && memcmp(seq[0], seq[1], sizeof(seq[0])) == 0 && memcmp(fx[0], fx[1], sizeof(fx[0])) == 0 && memcmp(fy[0], fy[1], sizeof(fy[0])) == 0;
    uint32_t gaps = 0;
    for (uint32_t i = 1; i < reads[1]; i++) {
        gaps += seq[1][i] != seq[1][i - 1] + 1;
    }
    CHECK(same && reads[0] >= 39, "E6: a drive across the 2^32 us wrap gives the same %u fixes as without it", reads[0]);
    CHECK(gaps == 0, "E6: a new fix at every read across the wrap: %u reads without one", gaps);
}

int main(void) {
    test_scene_round_trip();
    test_e1_at_rest();
    test_e2_drive_and_l8_speed();
    test_e3_station_dark();
    test_e4_late_duplicate();
    test_e5_moved();
    test_e6_wrap();
    printf("lh2_scene: %d passed, %d failed\n", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
