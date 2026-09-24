/**
 * @file
 * @brief  Host test of drv/steering, closed through the pose estimator and a simulated robot
 *
 * The robot follows its wheel setpoints through a first-order lag, turns in
 * place on a track that widens with spin speed as measured on carpet, covers
 * 2 per cent less ground than its encoders say, and is seen by LH2 at 10 Hz
 * with 2 ticks of age and 1 mm of noise. The steering sees it only through
 * drv/pose_estimator.
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "geometry.h"
#include "pose_estimator.h"
#include "protocol.h"
#include "steering.h"
#include "wheel_control.h"

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

#define DEG ((float)M_PI / 180.0f)

static const db_pose_estimator_conf_t _est_conf = {
    .lever_mm                     = DB_LH2_LEVER_ARM_EFFECTIVE,
    .lever_angle_deg              = DB_LH2_LEVER_ANGLE,
    .r_pos_mm2                    = DB_POSE_ESTIMATOR_R_POS_MM2,
    .q_pos_mm2_per_mm             = DB_POSE_ESTIMATOR_Q_POS_MM2_PER_MM,
    .q_heading_roll_deg2_per_mm   = DB_POSE_ESTIMATOR_Q_HEADING_ROLL_DEG2_PER_MM,
    .q_heading_turn_deg2_per_mm   = DB_POSE_ESTIMATOR_Q_HEADING_TURN_DEG2_PER_MM,
    .turn_speed_ref_mm_s          = DB_POSE_ESTIMATOR_TURN_SPEED_REF_MM_S,
    .gate                         = DB_POSE_ESTIMATOR_GATE,
    .fix_age_ticks                = DB_POSE_ESTIMATOR_FIX_AGE_TICKS,
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
};

static const db_steering_conf_t _conf = {
    .lever_mm              = DB_LH2_LEVER_ARM_EFFECTIVE,
    .v_max_mm_s            = DB_STEERING_V_MAX_MM_S,
    .approach_per_s        = DB_STEERING_APPROACH_PER_S,
    .runon_s               = DB_STEERING_RUNON_S,
    .spin_mm_s             = DB_STEERING_SPIN_MM_S,
    .spin_min_mm_s         = DB_STEERING_SPIN_MIN_MM_S,
    .heading_kp            = DB_STEERING_HEADING_KP,
    .heading_kd            = DB_STEERING_HEADING_KD,
    .align_enter_deg       = DB_STEERING_ALIGN_ENTER_DEG,
    .align_exit_deg        = DB_STEERING_ALIGN_EXIT_DEG,
    .full_speed_deg        = DB_STEERING_FULL_SPEED_DEG,
    .final_tol_deg         = DB_STEERING_FINAL_TOL_DEG,
    .near_mm               = DB_STEERING_NEAR_MM,
    .bearing_min_mm        = DB_STEERING_BEARING_MIN_MM,
    .lookahead_s           = DB_STEERING_LOOKAHEAD_S,
    .arrival_min_mm        = DB_STEERING_ARRIVAL_MIN_MM,
    .precise_min_mm        = DB_STEERING_PRECISE_MIN_MM,
    .pass_mm               = DB_STEERING_PASS_MM,
    .creep_mm_s            = DB_STEERING_CREEP_MM_S,
    .settle_skip_ticks     = DB_STEERING_SETTLE_SKIP_TICKS,
    .settle_fixes          = DB_STEERING_SETTLE_FIXES,
    .settle_ticks          = DB_STEERING_SETTLE_TICKS,
    .settle_nudges         = DB_STEERING_SETTLE_NUDGES,
    .nudge_ticks           = DB_STEERING_NUDGE_TICKS,
    .no_heading_turn_ticks = DB_STEERING_NO_HEADING_TURN_TICKS,
    .no_heading_ticks      = DB_STEERING_NO_HEADING_TICKS,
    .turn_ticks            = DB_STEERING_TURN_TICKS,
    .progress_ticks        = DB_STEERING_PROGRESS_TICKS,
    .progress_mm           = DB_STEERING_PROGRESS_MM,
    .hold_ticks            = DB_STEERING_HOLD_TICKS,
    .recover               = DB_STEERING_RECOVER_DRIVE,
    .recover_mm            = DB_STEERING_RECOVER_MM,
    .recover_mm_s          = DB_STEERING_RECOVER_MM_S,
    .bounds_margin_mm      = DB_STEERING_BOUNDS_MARGIN_MM,
};

//=========================== simulated robot ==================================

#define SUBSTEPS      (10)     ///< Integration steps per 10 ms tick
#define FIX_AGE_TICKS (2U)     ///< LH2 fix age, as measured on the floor
#define GROUND_SCALE  (0.98f)  ///< Ground covered per mm the encoders count
#define HISTORY       (8U)

typedef struct {
    float x;      ///< axle midpoint, mm
    float y;      ///< axle midpoint, mm
    float theta;  ///< rad, 0 along +y, clockwise positive
} pose_t;

typedef struct {
    float    x;              ///< axle midpoint, mm
    float    y;              ///< axle midpoint, mm
    float    theta;          ///< rad
    float    vl;             ///< actual left wheel speed, mm/s
    float    vr;             ///< actual right wheel speed, mm/s
    float    tau_s;          ///< wheel lag time constant, s
    float    count_l;        ///< encoder travel not yet counted, mm
    float    count_r;        ///< encoder travel not yet counted, mm
    uint32_t seed;           ///< noise generator
    uint32_t ticks;          ///< ticks run
    pose_t   hist[HISTORY];  ///< pose at each recent tick
    int      fixes;          ///< LH2 on
    int      blocked;        ///< wheels held still whatever the setpoint
    float    noise_mm;       ///< LH2 noise, sd
} robot_t;

typedef struct {
    robot_t              robot;
    db_pose_estimator_t  est;
    db_steering_t        steering;
    db_steering_output_t out;
    float                max_diff;           ///< largest |left - right| / 2 ever commanded, mm/s
    float                max_wheel;          ///< largest |wheel| ever commanded, mm/s
    uint32_t             arrived_tick;       ///< tick ARRIVED was first seen, 0 before
    int                  pivot_steps;        ///< outer steps commanded in place while turning
    int                  steps;              ///< outer steps
    float                pass_v_min;         ///< slowest commanded forward speed at an index change, mm/s
    float                approach_v_min;     ///< slowest commanded forward speed the step before an index change, mm/s
    float                last_v;             ///< commanded forward speed at the last step, mm/s
    float                first_settle_miss;  ///< true axle to the current point along the heading, at rest in the first SETTLE, mm
    float                max_v;              ///< fastest commanded forward speed, mm/s
    int                  settles;            ///< SETTLE entries
} sim_t;

static float _noise(robot_t *r) {
    r->seed  = r->seed * 1664525U + 1013904223U;
    float u1 = ((float)(r->seed >> 8) + 1.0f) / 16777217.0f;
    r->seed  = r->seed * 1664525U + 1013904223U;
    float u2 = (float)(r->seed >> 8) / 16777216.0f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

/// Track the robot really turns on: the spin track widens with spin speed
/// (81 at 100 mm/s per wheel, 83 at 200, 86.7 at 300 on carpet)
static float _track_true(float vl, float vr) {
    float spin  = fabsf(vl - vr) / 2.0f;
    float extra = 0;
    if (spin > 100.0f) {
        extra = (spin < 200.0f) ? 0.02f * (spin - 100.0f) : 2.0f + 0.037f * (spin - 200.0f);
    }
    return db_track_effective_mm(vl, vr) + extra;
}

static void _robot_tick(robot_t *r, float set_l, float set_r, int32_t *cl, int32_t *cr) {
    float dt = 0.01f / SUBSTEPS;
    float a  = dt / (r->tau_s + dt);
    for (int i = 0; i < SUBSTEPS; i++) {
        r->vl += a * ((r->blocked ? 0 : set_l) - r->vl);
        r->vr += a * ((r->blocked ? 0 : set_r) - r->vr);
        float dl = r->vl * dt;
        float dr = r->vr * dt;
        r->count_l += dl;
        r->count_r += dr;
        float d      = 0.5f * (dl + dr) * GROUND_SCALE;
        float dtheta = (dl - dr) * GROUND_SCALE / _track_true(r->vl, r->vr);
        float m      = r->theta + 0.5f * dtheta;
        r->x += -d * sinf(m);
        r->y += d * cosf(m);
        r->theta += dtheta;
    }
    *cl = (int32_t)(r->count_l / DB_MM_PER_COUNT);
    *cr = (int32_t)(r->count_r / DB_MM_PER_COUNT);
    r->count_l -= (float)*cl * DB_MM_PER_COUNT;
    r->count_r -= (float)*cr * DB_MM_PER_COUNT;
    r->hist[r->ticks % HISTORY] = (pose_t){ r->x, r->y, r->theta };
    r->ticks++;
}

static void _photodiode(float x, float y, float theta, float *px, float *py) {
    *px = x - DB_LH2_LEVER_ARM_EFFECTIVE * sinf(theta);
    *py = y + DB_LH2_LEVER_ARM_EFFECTIVE * cosf(theta);
}

static float _true_heading_deg(const sim_t *s) {
    float h = fmodf(s->robot.theta / DEG, 360.0f);
    if (h >= 180.0f) {
        h -= 360.0f;
    } else if (h < -180.0f) {
        h += 360.0f;
    }
    return h;
}

static float _angle_diff(float a, float b) {
    float e = fmodf(a - b, 360.0f);
    if (e >= 180.0f) {
        e -= 360.0f;
    } else if (e < -180.0f) {
        e += 360.0f;
    }
    return e;
}

/// A robot at a pose, with the estimator already tracking it there unless seeded is 0
static void _sim_init(sim_t *s, float x, float y, float heading_deg, int seeded, float tau_s) {
    *s                   = (sim_t){ 0 };
    s->robot.x           = x;
    s->robot.y           = y;
    s->robot.theta       = heading_deg * DEG;
    s->robot.tau_s       = tau_s;
    s->robot.seed        = 12345U;
    s->robot.fixes       = 1;
    s->robot.noise_mm    = 1.0f;
    s->pass_v_min        = INFINITY;
    s->approach_v_min    = INFINITY;
    s->first_settle_miss = -1;
    for (uint32_t i = 0; i < HISTORY; i++) {
        s->robot.hist[i] = (pose_t){ x, y, s->robot.theta };
    }
    db_pose_estimator_init(&s->est, &_est_conf);
    if (seeded) {
        db_pose_estimator_seed(&s->est, x, y, heading_deg, 1.0f);
    }
    db_steering_init(&s->steering, &_conf);
}

static void _pose_of(const db_pose_estimator_t *est, db_steering_pose_t *pose) {
    pose->status      = (est->status == DB_POSE_ESTIMATOR_TRACKING) ? DB_STEERING_POSE_TRACKING : ((est->status == DB_POSE_ESTIMATOR_LOST) ? DB_STEERING_POSE_LOST : DB_STEERING_POSE_SEEDING);
    pose->x_mm        = est->x;
    pose->y_mm        = est->y;
    pose->heading_deg = est->theta / DEG;
}

/// Run for a number of ticks: the robot and the estimator every tick, the
/// steering every DB_STEERING_PERIOD_TICKS and its poll every tick, as in the app
static void _sim_run(sim_t *s, uint32_t ticks) {
    for (uint32_t t = 0; t < ticks; t++) {
        robot_t           *r = &s->robot;
        db_steering_pose_t pose;
        if (r->ticks % DB_STEERING_PERIOD_TICKS == 0) {
            db_steering_state_t before = s->steering.state;
            uint8_t             index  = s->steering.index;
            _pose_of(&s->est, &pose);
            db_steering_step(&s->steering, &pose, DB_STEERING_PERIOD_TICKS, &s->out);
            s->steps++;
            if (!s->out.brake) {
                float diff   = fabsf(s->out.left_mm_s - s->out.right_mm_s) / 2.0f;
                float v      = (s->out.left_mm_s + s->out.right_mm_s) / 2.0f;
                s->max_diff  = fmaxf(s->max_diff, diff);
                s->max_wheel = fmaxf(s->max_wheel, fmaxf(fabsf(s->out.left_mm_s), fabsf(s->out.right_mm_s)));
                s->max_v     = fmaxf(s->max_v, fabsf(v));
                if (diff > 1.0f && fabsf(s->out.left_mm_s + s->out.right_mm_s) < 1e-3f) {
                    s->pivot_steps++;
                }
                if (s->steering.index != index && s->steering.index < s->steering.path.count) {
                    s->pass_v_min     = fminf(s->pass_v_min, fabsf(v));
                    s->approach_v_min = fminf(s->approach_v_min, fabsf(s->last_v));
                }
            }
            s->last_v = s->out.brake ? 0 : (s->out.left_mm_s + s->out.right_mm_s) / 2.0f;
            if (s->steering.state == DB_STEERING_SETTLE && before != DB_STEERING_SETTLE) {
                s->settles++;
            }
            if (s->steering.state == DB_STEERING_ARRIVED && s->arrived_tick == 0) {
                s->arrived_tick = r->ticks;
            }
        }
        float   set_l = s->out.brake ? 0 : s->out.left_mm_s;
        float   set_r = s->out.brake ? 0 : s->out.right_mm_s;
        int32_t cl, cr;
        _robot_tick(r, set_l, set_r, &cl, &cr);
        db_pose_estimator_predict(&s->est, cl, cr, 1);
        if (r->fixes && r->ticks % 10U == 0) {
            pose_t p = r->hist[(r->ticks - 1 - FIX_AGE_TICKS) % HISTORY];
            float  zx, zy;
            _photodiode(p.x, p.y, p.theta, &zx, &zy);
            zx += r->noise_mm * _noise(r);
            zy += r->noise_mm * _noise(r);
            db_pose_estimator_update(&s->est, zx, zy);
            db_steering_fix(&s->steering, zx, zy);
        }
        _pose_of(&s->est, &pose);
        if (db_steering_poll(&s->steering, &pose, &s->out) && s->steering.state == DB_STEERING_SETTLE) {
            s->settles++;
        }
        if (s->steering.state == DB_STEERING_SETTLE && s->steering.settle_count > 0 && s->first_settle_miss < 0) {
            s->first_settle_miss = fabsf(-(s->steering.target.x_mm - r->x) * sinf(r->theta) + (s->steering.target.y_mm - r->y) * cosf(r->theta));
        }
    }
}

/// Run until the steering stops moving or the time runs out; returns the ticks run
static uint32_t _sim_until_done(sim_t *s, uint32_t max_ticks) {
    uint32_t t = 0;
    while (t < max_ticks) {
        _sim_run(s, DB_STEERING_PERIOD_TICKS);
        t += DB_STEERING_PERIOD_TICKS;
        if (!db_steering_active(&s->steering)) {
            break;
        }
    }
    // let the run-on finish
    _sim_run(s, 50);
    return t;
}

static void _goto(sim_t *s, float x, float y, float threshold) {
    db_steering_target_t target = { .x_mm = x, .y_mm = y, .threshold_mm = threshold };
    db_steering_set_target(&s->steering, &target);
}

/// Distance of the true axle from a point
static float _miss(const sim_t *s, float x, float y) {
    return hypotf(s->robot.x - x, s->robot.y - y);
}

/// The robot faces +y from (1000, 500); a target at distance and bearing from its axle
static void _target_from(float distance, float bearing_deg, float *x, float *y) {
    *x = 1000.0f - distance * sinf(bearing_deg * DEG);
    *y = 500.0f + distance * cosf(bearing_deg * DEG);
}

//=========================== tests ============================================

static void test_idle_holds_still(void) {
    db_steering_t        steering;
    db_steering_output_t out;
    db_steering_pose_t   pose = { .status = DB_STEERING_POSE_TRACKING };
    db_steering_init(&steering, &_conf);
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(out.left_mm_s == 0 && out.right_mm_s == 0 && !out.brake, "IDLE commands nothing and does not brake");
    CHECK(!db_steering_active(&steering), "IDLE is not an active move");
}

static void test_target_ahead(void) {
    const float taus[] = { 0.02f, 0.04f, 0.05f };
    for (unsigned i = 0; i < sizeof(taus) / sizeof(taus[0]); i++) {
        sim_t s;
        float x, y;
        _sim_init(&s, 1000, 500, 0, 1, taus[i]);
        _target_from(600, 0, &x, &y);
        _goto(&s, x, y, 10);
        uint32_t t = _sim_until_done(&s, 1500);
        CHECK(s.steering.state == DB_STEERING_ARRIVED, "ahead, lag %.0f ms: arrived, state %d", taus[i] * 1000, s.steering.state);
        CHECK(_miss(&s, x, y) < 12.0f, "ahead, lag %.0f ms: stops within 12 mm, missed by %.1f", taus[i] * 1000, _miss(&s, x, y));
        CHECK(t < 400, "ahead, lag %.0f ms: 600 mm in under 4 s, took %u ticks", taus[i] * 1000, t);
        CHECK(s.pivot_steps == 0, "ahead, lag %.0f ms: no turn in place, %d steps", taus[i] * 1000, s.pivot_steps);
        CHECK(s.max_wheel <= DB_STEERING_V_MAX_MM_S + DB_STEERING_SPIN_MM_S, "ahead: wheel speed within cruise plus turn, %.0f", s.max_wheel);
    }
}

static void test_no_overshoot(void) {
    // Axle along the line of approach, tick by tick
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.05f);
    _target_from(800, 0, &x, &y);
    _goto(&s, x, y, 5);
    float past = -INFINITY;
    for (int t = 0; t < 1200; t++) {
        _sim_run(&s, 1);
        past = fmaxf(past, s.robot.y - y);
    }
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "800 mm at 300 mm/s: arrived, state %d", s.steering.state);
    CHECK(past < 5.0f, "the axle never passes the target by 5 mm, passed by %.1f", past);
}

static void test_target_behind_pivots(void) {
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _target_from(400, 180, &x, &y);
    _goto(&s, x, y, 10);
    _sim_run(&s, 10);
    CHECK(fabsf(s.out.left_mm_s + s.out.right_mm_s) < 1e-3f && fabsf(s.out.left_mm_s) > 1.0f, "behind: the first command turns in place, got %.0f %.0f", s.out.left_mm_s, s.out.right_mm_s);
    CHECK(s.steering.state == DB_STEERING_ALIGN, "behind: ALIGN first, state %d", s.steering.state);
    _sim_until_done(&s, 1500);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "behind: arrived, state %d", s.steering.state);
    CHECK(_miss(&s, x, y) < 12.0f, "behind: within 12 mm, missed by %.1f", _miss(&s, x, y));
    CHECK(fabsf(_angle_diff(_true_heading_deg(&s), 180)) < 10.0f, "behind: ends facing the way it drove, heading %.1f", _true_heading_deg(&s));
}

static void test_target_to_the_side(void) {
    const float bearings[] = { 90, -90, 45, -135 };
    for (unsigned i = 0; i < sizeof(bearings) / sizeof(bearings[0]); i++) {
        sim_t s;
        float x, y;
        _sim_init(&s, 1000, 500, 0, 1, 0.04f);
        _target_from(300, bearings[i], &x, &y);
        _goto(&s, x, y, 10);
        _sim_until_done(&s, 1500);
        CHECK(s.steering.state == DB_STEERING_ARRIVED, "bearing %.0f: arrived, state %d", bearings[i], s.steering.state);
        CHECK(_miss(&s, x, y) < 12.0f, "bearing %.0f: within 12 mm, missed by %.1f", bearings[i], _miss(&s, x, y));
        CHECK(s.pivot_steps > 0, "bearing %.0f: turns in place first, %d pivot steps", bearings[i], s.pivot_steps);
    }
}

static void test_very_close_targets(void) {
    // 25 mm ahead of the axle, and 25, 60 and 90 mm behind it
    const float distances[] = { 25, 25, 60, 90 };
    const float bearings[]  = { 0, 180, 180, 180 };
    for (unsigned i = 0; i < 4; i++) {
        sim_t s;
        float x, y;
        _sim_init(&s, 1000, 500, 0, 1, 0.04f);
        _target_from(distances[i], bearings[i], &x, &y);
        _goto(&s, x, y, 5);
        _sim_until_done(&s, 1000);
        CHECK(s.steering.state == DB_STEERING_ARRIVED, "%.0f mm at %.0f: arrived, state %d", distances[i], bearings[i], s.steering.state);
        CHECK(_miss(&s, x, y) < 7.0f, "%.0f mm at %.0f: within 7 mm, missed by %.1f", distances[i], bearings[i], _miss(&s, x, y));
        CHECK(fabsf(_true_heading_deg(&s)) < 15.0f, "%.0f mm at %.0f: backs up rather than turning round, heading %.1f", distances[i], bearings[i], _true_heading_deg(&s));
    }
}

static void test_near_axle_target_no_pivot(void) {
    // A target just behind the axle, a few mm off the heading line: the
    // robot gets there by backing up, not by turning round
    const float lateral[] = { 2, -3, 4, 4, 5, -6 };
    const float thr[]     = { 5, 10, 10, 5, 5, 5 };
    for (unsigned i = 0; i < 6; i++) {
        sim_t s;
        _sim_init(&s, 1000, 500, 0, 1, 0.04f);
        float x = 1000.0f + lateral[i], y = 500.0f - 55.0f;
        _goto(&s, x, y, thr[i]);
        _sim_until_done(&s, 1000);
        CHECK(s.steering.state == DB_STEERING_ARRIVED, "near axle, %.0f mm off the line: arrived, state %d", lateral[i], s.steering.state);
        if (fabsf(lateral[i]) < 0.5f * thr[i]) {
            CHECK(s.pivot_steps == 0 && fabsf(_true_heading_deg(&s)) < 10.0f, "near axle, %.0f mm off the line, threshold %.0f: drives along the line, %d pivot steps, heading %.1f", lateral[i], thr[i], s.pivot_steps, _true_heading_deg(&s));
        } else {
            CHECK(fabsf(_true_heading_deg(&s)) < 45.0f, "near axle, %.0f mm off the line, threshold %.0f: turns a little, not round, heading %.1f", lateral[i], thr[i], _true_heading_deg(&s));
        }
        CHECK(_miss(&s, x, y) <= thr[i] + 2.0f, "near axle, %.0f mm off the line: within the threshold, missed by %.1f", lateral[i], _miss(&s, x, y));
    }
}

static void test_overshoot_reverses(void) {
    // Driving at 200 mm/s with the target already 30 or 80 mm behind the axle
    const float behind[] = { 30, 80 };
    for (unsigned i = 0; i < 2; i++) {
        sim_t s;
        float x, y;
        _sim_init(&s, 1000, 500, 0, 1, 0.04f);
        s.robot.vl = s.robot.vr = 200.0f;
        _target_from(behind[i], 180, &x, &y);
        _goto(&s, x, y, 5);
        s.steering.state  = DB_STEERING_DRIVE;
        s.steering.v_mm_s = 200.0f;
        _sim_until_done(&s, 1000);
        CHECK(s.steering.state == DB_STEERING_ARRIVED, "overshot by %.0f mm: arrived, state %d", behind[i], s.steering.state);
        CHECK(s.pivot_steps == 0, "overshot by %.0f mm: reverses, no turn in place, %d pivot steps", behind[i], s.pivot_steps);
        CHECK(fabsf(_true_heading_deg(&s)) < 10.0f, "overshot by %.0f mm: still facing the way it came, heading %.1f", behind[i], _true_heading_deg(&s));
        CHECK(_miss(&s, x, y) < 7.0f, "overshot by %.0f mm: within 7 mm, missed by %.1f", behind[i], _miss(&s, x, y));
    }
}

static void test_already_there(void) {
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _target_from(3, 90, &x, &y);
    _goto(&s, x, y, 10);
    _sim_run(&s, 10);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "a target inside the threshold arrives at once, state %d", s.steering.state);
    CHECK(s.out.brake, "and brakes");
}

static void test_arrival_latches(void) {
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _target_from(300, 30, &x, &y);
    _goto(&s, x, y, 10);
    _sim_until_done(&s, 1500);
    float x0 = s.robot.x, y0 = s.robot.y, h0 = s.robot.theta;
    int   moved = 0;
    for (int i = 0; i < 50; i++) {
        _sim_run(&s, 10);
        moved |= !s.out.brake || s.out.left_mm_s != 0 || s.out.right_mm_s != 0 || s.steering.state != DB_STEERING_ARRIVED;
    }
    CHECK(!moved, "ARRIVED stays latched, braked, for 5 s");
    CHECK(hypotf(s.robot.x - x0, s.robot.y - y0) < 0.1f && fabsf(s.robot.theta - h0) < 0.001f, "and the robot does not move");
    CHECK(!db_steering_active(&s.steering), "ARRIVED is not an active move");
}

static void test_final_heading(void) {
    const float headings[] = { 90, -150, 0 };
    for (unsigned i = 0; i < 3; i++) {
        sim_t s;
        float x, y;
        _sim_init(&s, 1000, 500, 0, 1, 0.04f);
        _target_from(400, 45, &x, &y);
        db_steering_target_t target = { .x_mm = x, .y_mm = y, .threshold_mm = 10, .has_final_heading = true, .final_heading_deg = headings[i] };
        db_steering_set_target(&s.steering, &target);
        int turned = 0;
        for (int t = 0; t < 200 && db_steering_active(&s.steering); t++) {
            _sim_run(&s, 10);
            turned |= s.steering.state == DB_STEERING_FINAL_TURN;
        }
        _sim_run(&s, 50);
        CHECK(turned, "final heading %.0f: goes through FINAL_TURN", headings[i]);
        CHECK(s.steering.state == DB_STEERING_ARRIVED, "final heading %.0f: arrived, state %d", headings[i], s.steering.state);
        CHECK(fabsf(_angle_diff(_true_heading_deg(&s), headings[i])) < 5.0f, "final heading %.0f: within 5 deg, true %.1f", headings[i], _true_heading_deg(&s));
        CHECK(hypotf(s.robot.x - x, s.robot.y - y) < 12.0f, "final heading %.0f: axle within 12 mm of the pose after the turn, missed by %.1f", headings[i], hypotf(s.robot.x - x, s.robot.y - y));
    }
}

static void test_no_heading_spins_then_drives(void) {
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 70, 0, 0.04f);
    _target_from(400, 0, &x, &y);
    _goto(&s, x, y, 10);
    _sim_run(&s, 10);
    CHECK(s.steering.state == DB_STEERING_NO_HEADING, "no heading: NO_HEADING, state %d", s.steering.state);
    CHECK(s.out.left_mm_s == DB_STEERING_SPIN_MM_S && s.out.right_mm_s == -DB_STEERING_SPIN_MM_S, "no heading: spins in place at the trusted limit, %.0f %.0f", s.out.left_mm_s, s.out.right_mm_s);
    int spun = 0;
    while (s.steering.state == DB_STEERING_NO_HEADING && spun < 400) {
        _sim_run(&s, 10);
        spun += 10;
    }
    CHECK(spun + 10 >= (int)DB_STEERING_NO_HEADING_TURN_TICKS && spun < 150, "no heading: one full turn, not stopping at acquisition, %d ticks", spun);
    _sim_until_done(&s, 1500);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "no heading: then arrives, state %d", s.steering.state);
    CHECK(_miss(&s, x, y) < 12.0f, "no heading: within 12 mm, missed by %.1f", _miss(&s, x, y));
}

static void test_no_heading_times_out(void) {
    sim_t s;
    _sim_init(&s, 1000, 500, 0, 0, 0.04f);
    s.robot.fixes = 0;
    _goto(&s, 1000, 900, 10);
    _sim_until_done(&s, 1000);
    CHECK(s.steering.state == DB_STEERING_FAILED && s.steering.fail == DB_STEERING_FAIL_NO_HEADING, "no fixes: NO_HEADING fails, state %d fail %d", s.steering.state, s.steering.fail);
    CHECK(s.out.brake, "and brakes");
}

static void test_lost_holds_then_resumes(void) {
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _target_from(700, 0, &x, &y);
    _goto(&s, x, y, 10);
    _sim_run(&s, 50);
    s.robot.fixes = 0;
    int held      = 0;
    for (int t = 0; t < 200; t += 10) {
        _sim_run(&s, 10);
        held |= s.steering.state == DB_STEERING_HOLD;
    }
    CHECK(held && s.steering.state == DB_STEERING_HOLD, "no fixes for 2 s: HOLD, state %d", s.steering.state);
    CHECK(s.out.brake, "HOLD brakes");
    float x0 = s.robot.x, y0 = s.robot.y;
    _sim_run(&s, 50);
    CHECK(hypotf(s.robot.x - x0, s.robot.y - y0) < 0.1f, "the robot stands in HOLD");
    CHECK(db_steering_active(&s.steering), "HOLD is still an active move");
    s.robot.fixes = 1;
    _sim_until_done(&s, 1500);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "fixes back: resumes and arrives, state %d", s.steering.state);
    CHECK(_miss(&s, x, y) < 12.0f, "fixes back: within 12 mm, missed by %.1f", _miss(&s, x, y));
}

static void test_lost_for_good_fails(void) {
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _target_from(700, 0, &x, &y);
    _goto(&s, x, y, 10);
    _sim_run(&s, 50);
    s.robot.fixes = 0;
    _sim_run(&s, 800);
    CHECK(s.steering.state == DB_STEERING_FAILED && s.steering.fail == DB_STEERING_FAIL_HOLD, "no fixes for good: FAILED after the HOLD timeout, state %d fail %d", s.steering.state, s.steering.fail);
}

static void test_heading_lost_mid_move_recovers_straight(void) {
    // Unit level: SEEDING while driving drives straight on, and gives up after recover_mm
    db_steering_t        steering;
    db_steering_output_t out;
    db_steering_pose_t   pose   = { .status = DB_STEERING_POSE_TRACKING, .x_mm = 1000, .y_mm = 500, .heading_deg = 0 };
    db_steering_target_t target = { .x_mm = 1000, .y_mm = 1000, .threshold_mm = 10 };
    db_steering_init(&steering, &_conf);
    db_steering_set_target(&steering, &target);
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.state == DB_STEERING_DRIVE, "aimed at the target: DRIVE at once, state %d", steering.state);
    pose.status = DB_STEERING_POSE_SEEDING;
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.state == DB_STEERING_RECOVER && out.left_mm_s == DB_STEERING_RECOVER_MM_S && out.right_mm_s == DB_STEERING_RECOVER_MM_S,
          "heading lost while driving: straight on at the recovery speed, state %d, %.0f %.0f", steering.state, out.left_mm_s, out.right_mm_s);
    int steps = 0;
    while (steering.state == DB_STEERING_RECOVER && steps < 20) {
        db_steering_step(&steering, &pose, 10, &out);
        steps++;
    }
    float travelled = (float)steps * 0.1f * DB_STEERING_RECOVER_MM_S;
    CHECK(steering.state == DB_STEERING_FAILED && steering.fail == DB_STEERING_FAIL_HEADING_LOST, "not re-acquired: FAILED, state %d fail %d", steering.state, steering.fail);
    CHECK(travelled <= DB_STEERING_RECOVER_MM + 0.1f * DB_STEERING_RECOVER_MM_S, "after at most the recovery distance, %.0f mm", travelled);
    CHECK(out.brake && out.left_mm_s == 0 && out.right_mm_s == 0, "and brakes");

    // From rest, the same loss spins instead
    db_steering_conf_t spin = _conf;
    spin.recover            = DB_STEERING_RECOVER_SPIN;
    db_steering_init(&steering, &spin);
    db_steering_set_target(&steering, &target);
    pose.status = DB_STEERING_POSE_TRACKING;
    db_steering_step(&steering, &pose, 10, &out);
    pose.status = DB_STEERING_POSE_SEEDING;
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.state == DB_STEERING_NO_HEADING, "recover = SPIN: back to NO_HEADING, state %d", steering.state);
}

/// Drives toward a target far ahead, then drops the estimator to SEEDING mid-drive
static void _lose_heading_mid_drive(sim_t *s, const db_steering_conf_t *conf, int fixes_after) {
    float x, y;
    _sim_init(s, 1000, 300, 0, 1, 0.04f);
    s->steering.conf = conf;
    _target_from(700, 0, &x, &y);
    _goto(s, x, y, 10);
    _sim_run(s, 100);
    s->est.status      = DB_POSE_ESTIMATOR_SEEDING;
    s->est.chain_count = 0;
    s->robot.fixes     = fixes_after;
}

static void test_recover_straight_closed_loop(void) {
    sim_t s;
    float x, y;
    _target_from(700, 0, &x, &y);
    _lose_heading_mid_drive(&s, &_conf, 1);
    float x0 = s.robot.x, y0 = s.robot.y;
    int   recovering = 0, spun = 0;
    float at_reacquire = -1;
    for (int t = 0; t < 1500 && db_steering_active(&s.steering); t += 10) {
        _sim_run(&s, 10);
        if (s.steering.state == DB_STEERING_RECOVER) {
            recovering = 1;
        } else if (recovering && at_reacquire < 0) {
            at_reacquire = hypotf(s.robot.x - x0, s.robot.y - y0);
        }
        spun |= s.steering.state == DB_STEERING_NO_HEADING;
    }
    _sim_run(&s, 50);
    CHECK(recovering && !spun, "heading lost mid-drive: straight on, no spin, recovered %d spun %d", recovering, spun);
    CHECK(at_reacquire > 0 && at_reacquire < 100.0f, "re-acquired within 100 mm of travel from 300 mm/s, %.1f", at_reacquire);
    CHECK(s.steering.state == DB_STEERING_ARRIVED && _miss(&s, x, y) < 12.0f, "then arrives, state %d, missed by %.1f", s.steering.state, _miss(&s, x, y));

    // No fixes: gives up after the recovery distance
    _lose_heading_mid_drive(&s, &_conf, 0);
    x0 = s.robot.x;
    y0 = s.robot.y;
    _sim_run(&s, 200);
    CHECK(s.steering.state == DB_STEERING_FAILED && s.steering.fail == DB_STEERING_FAIL_HEADING_LOST, "no fixes: FAILED, state %d fail %d", s.steering.state, s.steering.fail);
    CHECK(hypotf(s.robot.x - x0, s.robot.y - y0) < DB_STEERING_RECOVER_MM + 40.0f, "having gone no further than the recovery straight and its run-on, %.1f mm", hypotf(s.robot.x - x0, s.robot.y - y0));

    // Bounds just ahead: spins instead
    db_steering_conf_t bounded = _conf;
    bounded.bounds_mm[0]       = 0;
    bounded.bounds_mm[1]       = 0;
    bounded.bounds_mm[2]       = 2000;
    bounded.bounds_mm[3]       = 1;
    _lose_heading_mid_drive(&s, &bounded, 1);
    bounded.bounds_mm[3] = s.steering.last_y_mm + 120.0f;
    _sim_run(&s, 10);
    CHECK(s.steering.state == DB_STEERING_NO_HEADING, "the straight would leave the bounds: spins instead, state %d", s.steering.state);
    _sim_until_done(&s, 1500);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "and then arrives, state %d", s.steering.state);
}

static void test_new_target_mid_drive(void) {
    sim_t s;
    float x, y, x2, y2;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _target_from(700, 0, &x, &y);
    _target_from(500, 5, &x2, &y2);
    _goto(&s, x, y, 10);
    _sim_run(&s, 100);
    CHECK(s.steering.state == DB_STEERING_DRIVE, "driving before the new target, state %d", s.steering.state);
    _goto(&s, x2, y2, 10);
    _sim_run(&s, 10);
    int pivots = s.pivot_steps;
    CHECK(s.steering.state == DB_STEERING_DRIVE, "a new target a few degrees off keeps driving, state %d", s.steering.state);
    _sim_until_done(&s, 1500);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "new target: arrived, state %d", s.steering.state);
    CHECK(_miss(&s, x2, y2) < 12.0f, "new target: within 12 mm of it, missed by %.1f", _miss(&s, x2, y2));
    CHECK(s.pivot_steps == pivots, "new target: no stop to turn in place, %d pivot steps", s.pivot_steps - pivots);
}

/// Steering state driving toward (1000, 1000) from (1000, 200) at cruise, facing it
static void _cruising(db_steering_t *steering, db_steering_output_t *out) {
    db_steering_pose_t   pose   = { .status = DB_STEERING_POSE_TRACKING, .x_mm = 1000, .y_mm = 200, .heading_deg = 0 };
    db_steering_target_t target = { .x_mm = 1000, .y_mm = 1000, .threshold_mm = 20 };
    db_steering_init(steering, &_conf);
    db_steering_set_target(steering, &target);
    db_steering_step(steering, &pose, 10, out);
}

static void test_speed_falls_with_heading_error(void) {
    db_steering_t        steering;
    db_steering_output_t out;
    _cruising(&steering, &out);
    CHECK(fabsf((out.left_mm_s + out.right_mm_s) / 2.0f - DB_STEERING_V_MAX_MM_S) < 1.0f, "aimed and far: cruise, %.0f", (out.left_mm_s + out.right_mm_s) / 2.0f);
    // 15 deg off, with no turn in the prediction: a third of the approach speed
    db_steering_pose_t pose = { .status = DB_STEERING_POSE_TRACKING, .x_mm = 1000, .y_mm = 200, .heading_deg = 15 };
    steering.omega_deg_s    = 0;
    db_steering_step(&steering, &pose, 10, &out);
    float v = (out.left_mm_s + out.right_mm_s) / 2.0f;
    CHECK(steering.state == DB_STEERING_DRIVE, "15 deg off: still DRIVE, state %d", steering.state);
    CHECK(fabsf(v - DB_STEERING_V_MAX_MM_S / 3.0f) < 10.0f, "15 deg off: a third of cruise, %.0f", v);
}

static void test_arrival_counts_the_runon(void) {
    // At cruise the robot runs on 15 mm, so an axle 30 mm short of a
    // 20 mm threshold is not arrived and one 10 mm short is
    db_steering_t        steering;
    db_steering_output_t out;
    _cruising(&steering, &out);
    float              axle_y = 1000.0f - 20.0f - 30.0f;
    db_steering_pose_t pose   = { .status = DB_STEERING_POSE_TRACKING, .x_mm = 1000, .y_mm = axle_y, .heading_deg = 0 };
    steering.v_mm_s           = DB_STEERING_V_MAX_MM_S;
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.state == DB_STEERING_DRIVE, "30 mm short at cruise: still driving, state %d", steering.state);
    pose.y_mm       = axle_y + 20.0f;
    steering.v_mm_s = DB_STEERING_V_MAX_MM_S;
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.state == DB_STEERING_ARRIVED && out.brake, "10 mm short at cruise: arrived, braking, state %d", steering.state);
}

static void test_turn_is_predicted(void) {
    // 12 deg off, turning at 100 deg/s: the heading 50 ms on is 7 deg off, below the exit
    db_steering_t        steering;
    db_steering_output_t out;
    db_steering_pose_t   pose   = { .status = DB_STEERING_POSE_TRACKING, .x_mm = 1000, .y_mm = 200, .heading_deg = -12 };
    db_steering_target_t target = { .x_mm = 1000, .y_mm = 1000, .threshold_mm = 20 };
    db_steering_init(&steering, &_conf);
    db_steering_set_target(&steering, &target);
    steering.state       = DB_STEERING_ALIGN;
    steering.omega_deg_s = 100.0f;
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.state == DB_STEERING_DRIVE, "ALIGN ends on the predicted heading, state %d", steering.state);
}

static void test_retarget_after_arrival(void) {
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _target_from(200, 0, &x, &y);
    _goto(&s, x, y, 10);
    _sim_until_done(&s, 1000);
    _goto(&s, 900, 500, 10);
    CHECK(db_steering_active(&s.steering), "a new target re-arms a latched arrival");
    _sim_until_done(&s, 1500);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "and the second target arrives, state %d", s.steering.state);
    CHECK(_miss(&s, 900, 500) < 12.0f, "within 12 mm, missed by %.1f", _miss(&s, 900, 500));
}

static void test_blocked_wheels_fail_on_progress(void) {
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _target_from(500, 0, &x, &y);
    _goto(&s, x, y, 10);
    s.robot.blocked = 1;
    _sim_until_done(&s, 1000);
    CHECK(s.steering.state == DB_STEERING_FAILED && s.steering.fail == DB_STEERING_FAIL_PROGRESS, "no progress: FAILED, state %d fail %d", s.steering.state, s.steering.fail);
}

static void test_turn_never_exceeds_spin_limit(void) {
    // Every closed-loop scenario above records its largest commanded turn; run a
    // few hard ones here and check them all
    float       worst       = 0;
    const float bearings[]  = { 180, 120, -100, 170 };
    const float distances[] = { 500, 150, 800, 90 };
    for (unsigned i = 0; i < 4; i++) {
        sim_t s;
        float x, y;
        _sim_init(&s, 1000, 500, 0, 1, 0.04f);
        _target_from(distances[i], bearings[i], &x, &y);
        _goto(&s, x, y, 10);
        _sim_until_done(&s, 1500);
        worst = fmaxf(worst, s.max_diff);
    }
    CHECK(worst <= DB_STEERING_SPIN_MM_S + 1e-3f, "turn stays within the trusted spin limit, worst %.1f mm/s per wheel", worst);
    CHECK(worst >= DB_STEERING_SPIN_MM_S - 1e-3f, "and a large error does use it, worst %.1f", worst);
}

static void test_pivot_without_floor(void) {
    db_steering_t        steering;
    db_steering_output_t out;
    db_steering_pose_t   pose   = { .status = DB_STEERING_POSE_TRACKING, .x_mm = 1000, .y_mm = 500, .heading_deg = 0 };
    db_steering_target_t target = { .x_mm = 1500, .y_mm = 500, .threshold_mm = 10 };
    db_steering_init(&steering, &_conf);
    db_steering_set_target(&steering, &target);
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.state == DB_STEERING_ALIGN, "a target 90 deg off: ALIGN, state %d", steering.state);
    CHECK(out.left_mm_s == -out.right_mm_s && out.left_mm_s < 0, "turns in place counter-clockwise with no forward speed, %.1f %.1f", out.left_mm_s, out.right_mm_s);
    CHECK(fabsf(out.left_mm_s) == DB_STEERING_SPIN_MM_S, "at the spin limit, %.1f", out.left_mm_s);

    // A small residual error in place still turns, at the least useful rate
    target.has_final_heading = true;
    target.final_heading_deg = 4.0f;
    target.x_mm              = 1000.0f;
    target.y_mm              = 500.0f;
    db_steering_init(&steering, &_conf);
    db_steering_set_target(&steering, &target);
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.state == DB_STEERING_FINAL_TURN, "final heading 4 deg off at the target: FINAL_TURN, state %d", steering.state);
    CHECK(out.left_mm_s == DB_STEERING_SPIN_MIN_MM_S && out.right_mm_s == -DB_STEERING_SPIN_MIN_MM_S, "turns at the least useful rate, %.1f %.1f", out.left_mm_s, out.right_mm_s);
}

static void test_stop_goes_idle(void) {
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _target_from(500, 0, &x, &y);
    _goto(&s, x, y, 10);
    _sim_run(&s, 50);
    db_steering_stop(&s.steering);
    _sim_run(&s, 10);
    CHECK(s.steering.state == DB_STEERING_IDLE && s.out.left_mm_s == 0 && s.out.right_mm_s == 0 && !s.out.brake, "stop: IDLE, zero setpoints, no latched brake");
}

//=========================== batches ==========================================

/// Start a batch of points (x, y), none with a heading
static void _path(sim_t *s, const float pts[][2], int n, float threshold, float pass) {
    db_steering_path_t path = { .count = (uint8_t)n, .threshold_mm = threshold, .pass_mm = pass };
    for (int i = 0; i < n; i++) {
        path.points[i].x_mm = pts[i][0];
        path.points[i].y_mm = pts[i][1];
    }
    db_steering_set_path(&s->steering, &path);
}

static float _segment_distance(float x, float y, float ax, float ay, float bx, float by) {
    float ux = bx - ax, uy = by - ay;
    float len2 = ux * ux + uy * uy;
    float t    = (len2 > 0) ? ((x - ax) * ux + (y - ay) * uy) / len2 : 0;
    t          = (t < 0) ? 0 : ((t > 1) ? 1 : t);
    return hypotf(x - ax - t * ux, y - ay - t * uy);
}

/// What a batch run did
typedef struct {
    uint32_t ticks;           ///< ticks until the steering stopped
    float    worst_off_path;  ///< farthest the true axle strayed from the polyline, mm
    int      early_brakes;    ///< steps braked before the last point
    uint8_t  indices[64];     ///< the index at each change
    int      n_indices;       ///< changes recorded
} run_t;

/// Run a batch from where the robot stands to its end, measuring the true axle
/// against the polyline from the start through every point
static run_t _run_path(sim_t *s, const float pts[][2], int n, uint32_t max_ticks) {
    run_t r  = { 0 };
    float x0 = s->robot.x, y0 = s->robot.y;
    r.indices[r.n_indices++] = s->steering.index;
    while (r.ticks < max_ticks && db_steering_active(&s->steering)) {
        _sim_run(s, 1);
        r.ticks++;
        float best = hypotf(s->robot.x - x0, s->robot.y - y0);
        float ax = x0, ay = y0;
        for (int i = 0; i < n; i++) {
            best = fminf(best, _segment_distance(s->robot.x, s->robot.y, ax, ay, pts[i][0], pts[i][1]));
            ax   = pts[i][0];
            ay   = pts[i][1];
        }
        r.worst_off_path = fmaxf(r.worst_off_path, best);
        if (s->out.brake && s->steering.index + 1U < s->steering.path.count) {
            r.early_brakes++;
        }
        if (s->steering.index != r.indices[r.n_indices - 1] && r.n_indices < 64) {
            r.indices[r.n_indices++] = s->steering.index;
        }
    }
    _sim_run(s, 50);
    return r;
}

static int _indices_count_up(const run_t *r, int count) {
    for (int i = 0; i < r->n_indices; i++) {
        if (r->indices[i] != i) {
            return 0;
        }
    }
    return r->n_indices == count + 1;
}

static void test_square(void) {
    const float pts[][2] = { { 1000, 800 }, { 1300, 800 }, { 1300, 500 }, { 1000, 500 } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _path(&s, pts, 4, 10, 20);
    run_t r = _run_path(&s, pts, 4, 2000);
    CHECK(s.steering.state == DB_STEERING_ARRIVED && s.steering.completion == DB_STEERING_DONE_ARRIVED, "square: arrived, state %d", s.steering.state);
    CHECK(_miss(&s, 1000, 500) < 12.0f, "square: axle within 12 mm of the last point, missed by %.1f", _miss(&s, 1000, 500));
    CHECK(_indices_count_up(&r, 4), "square: index 0, 1, 2, 3 then 4, %d changes", r.n_indices);
    CHECK(r.early_brakes == 0, "square: no brake before the last point, %d steps", r.early_brakes);
    CHECK(s.pivot_steps > 0, "square: turns in place at the 90 degree corners, %d pivot steps", s.pivot_steps);
    CHECK(r.worst_off_path < 25.0f, "square: axle within 25 mm of the polyline, %.1f", r.worst_off_path);
    CHECK(r.ticks < 800, "square: 1.2 m in under 8 s, %u ticks", r.ticks);
}

static void test_zigzag(void) {
    const float pts[][2] = { { 1100, 700 }, { 900, 900 }, { 1100, 1100 }, { 900, 1300 } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _path(&s, pts, 4, 10, 20);
    run_t r = _run_path(&s, pts, 4, 2500);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "zigzag: arrived, state %d", s.steering.state);
    CHECK(_miss(&s, 900, 1300) < 12.0f, "zigzag: within 12 mm of the last point, missed by %.1f", _miss(&s, 900, 1300));
    CHECK(_indices_count_up(&r, 4), "zigzag: index counts up, %d changes", r.n_indices);
    CHECK(r.early_brakes == 0, "zigzag: no brake before the last point, %d", r.early_brakes);
    CHECK(r.worst_off_path < 30.0f, "zigzag: axle within 30 mm of the polyline, %.1f", r.worst_off_path);
}

static void test_gentle_curve_keeps_speed(void) {
    // 10 degrees of turn every 150 mm
    float pts[6][2];
    float x = 1000, y = 500, h = 0;
    for (int i = 0; i < 6; i++) {
        h -= 10.0f;
        x += -150.0f * sinf(h * DEG);
        y += 150.0f * cosf(h * DEG);
        pts[i][0] = x;
        pts[i][1] = y;
    }
    sim_t s;
    _sim_init(&s, 1000, 500, -10, 1, 0.04f);  // facing the first point
    _path(&s, (const float(*)[2])pts, 6, 10, 20);
    run_t r = _run_path(&s, (const float(*)[2])pts, 6, 2000);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "curve: arrived, state %d", s.steering.state);
    CHECK(s.pivot_steps == 0, "curve: never turns in place, %d pivot steps", s.pivot_steps);
    CHECK(s.pass_v_min > 80.0f, "curve: never slows below 80 mm/s through a point, slowest %.0f", s.pass_v_min);
    CHECK(r.worst_off_path < 15.0f, "curve: axle within 15 mm of the polyline, %.1f", r.worst_off_path);
    CHECK(r.ticks < 500, "curve: 900 mm in under 5 s, %u ticks", r.ticks);
}

static void test_sharp_corner_slows(void) {
    // The approach to a 90 degree corner keeps no exit speed, one of 10 degrees keeps it all
    const float square[][2] = { { 1000, 900 }, { 1400, 900 } };
    const float gentle[][2] = { { 1000, 900 }, { 1000.0f + 400.0f * 0.17365f, 900.0f + 400.0f * 0.98481f } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _path(&s, square, 2, 10, 20);
    _run_path(&s, square, 2, 1500);
    float sharp = s.pass_v_min, sharp_approach = s.approach_v_min;
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "90 degree corner: arrived, state %d", s.steering.state);
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _path(&s, gentle, 2, 10, 20);
    _run_path(&s, gentle, 2, 1500);
    CHECK(sharp < 1.0f && s.pass_v_min > 150.0f, "passes a 90 degree corner slowly and a 10 degree one fast, %.0f and %.0f mm/s", sharp, s.pass_v_min);
    CHECK(sharp_approach < 120.0f && s.approach_v_min > 250.0f, "slows into the 90 degree corner, not the 10 degree one, %.0f and %.0f mm/s", sharp_approach, s.approach_v_min);
}

static void test_u_turn(void) {
    const float pts[][2] = { { 1000, 900 }, { 1000, 500 } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _path(&s, pts, 2, 10, 20);
    run_t r = _run_path(&s, pts, 2, 2000);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "U-turn: arrived, state %d", s.steering.state);
    CHECK(_miss(&s, 1000, 500) < 12.0f, "U-turn: back within 12 mm, missed by %.1f", _miss(&s, 1000, 500));
    CHECK(fabsf(_angle_diff(_true_heading_deg(&s), 180)) < 15.0f, "U-turn: turned round, heading %.1f", _true_heading_deg(&s));
    CHECK(r.worst_off_path < 25.0f, "U-turn: axle within 25 mm of the line, %.1f", r.worst_off_path);
}

static void test_point_behind(void) {
    // The second point is 60 mm behind the first: backed up to, not turned round for
    const float pts[][2] = { { 1000, 800 }, { 1000, 740 }, { 1300, 740 } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _path(&s, pts, 3, 10, 20);
    run_t r = _run_path(&s, pts, 3, 2000);
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "point behind: arrived, state %d", s.steering.state);
    CHECK(_indices_count_up(&r, 3), "point behind: index counts up, %d changes", r.n_indices);
    CHECK(_miss(&s, 1300, 740) < 12.0f, "point behind: within 12 mm of the last point, missed by %.1f", _miss(&s, 1300, 740));
}

static void test_completion(void) {
    const float pts[][2] = { { 1000, 700 }, { 1000, 900 } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    CHECK(s.steering.completion == DB_STEERING_DONE_NONE, "no batch yet: NONE, %d", s.steering.completion);
    _path(&s, pts, 2, 10, 20);
    CHECK(s.steering.completion == DB_STEERING_DONE_IN_PROGRESS && s.steering.index == 0, "batch set: IN_PROGRESS at index 0");
    _sim_until_done(&s, 1500);
    CHECK(s.steering.completion == DB_STEERING_DONE_ARRIVED && s.steering.index == 2, "ARRIVED with index at the count, %d %d", s.steering.completion, s.steering.index);
    db_steering_stop(&s.steering);
    CHECK(s.steering.completion == DB_STEERING_DONE_ARRIVED, "a stop after arriving keeps ARRIVED, %d", s.steering.completion);

    _path(&s, pts, 2, 10, 20);
    _sim_run(&s, 30);
    db_steering_stop(&s.steering);
    CHECK(s.steering.completion == DB_STEERING_DONE_ABORTED, "a stop in progress: ABORTED, %d", s.steering.completion);
}

static void test_stop_keeps_the_outcome(void) {
    // A stop after the batch has ended keeps its status and its reason
    const float pts[][2] = { { 1000, 900 } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _path(&s, pts, 1, 10, 0);
    _sim_run(&s, 30);
    s.robot.blocked = 1;
    _sim_until_done(&s, 1500);
    CHECK(s.steering.completion == DB_STEERING_DONE_FAILED && s.steering.fail == DB_STEERING_FAIL_PROGRESS, "blocked: FAILED on progress, %d %d", s.steering.completion, s.steering.fail);
    db_steering_stop(&s.steering);
    CHECK(s.steering.state == DB_STEERING_IDLE && s.steering.completion == DB_STEERING_DONE_FAILED && s.steering.fail == DB_STEERING_FAIL_PROGRESS,
          "a stop after failing keeps FAILED and its reason, %d %d", s.steering.completion, s.steering.fail);
    s.robot.blocked = 0;
    _path(&s, pts, 1, 10, 0);
    CHECK(s.steering.completion == DB_STEERING_DONE_IN_PROGRESS && s.steering.fail == DB_STEERING_FAIL_NONE, "a new batch clears the reason, %d %d", s.steering.completion, s.steering.fail);
}

static void test_empty_path_stops(void) {
    const float pts[][2] = { { 1000, 900 } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _path(&s, pts, 1, 10, 0);
    _sim_run(&s, 30);
    _path(&s, pts, 0, 10, 0);
    _sim_run(&s, 10);
    CHECK(s.steering.state == DB_STEERING_IDLE && s.steering.completion == DB_STEERING_DONE_ABORTED && !s.out.brake && s.out.left_mm_s == 0 && s.out.right_mm_s == 0,
          "an empty batch in progress: IDLE, ABORTED, zero setpoints, state %d completion %d", s.steering.state, s.steering.completion);
}

static void test_failed_mid_path(void) {
    const float pts[][2] = { { 1000, 700 }, { 1000, 900 }, { 1000, 1100 } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _path(&s, pts, 3, 10, 20);
    for (int t = 0; t < 1500 && s.steering.index < 1; t += 10) {
        _sim_run(&s, 10);
    }
    s.robot.blocked = 1;
    _sim_until_done(&s, 1500);
    CHECK(s.steering.state == DB_STEERING_FAILED && s.steering.completion == DB_STEERING_DONE_FAILED && s.steering.fail == DB_STEERING_FAIL_PROGRESS,
          "blocked after the first point: FAILED on progress, state %d fail %d", s.steering.state, s.steering.fail);
    CHECK(s.steering.index == 1, "and reports the point it was driving to, index %d", s.steering.index);
    CHECK(s.out.brake, "and brakes");
}

static void test_retarget_mid_path(void) {
    const float first[][2]  = { { 1000, 800 }, { 1300, 800 }, { 1300, 500 } };
    const float second[][2] = { { 1400, 900 }, { 1400, 1100 } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    _path(&s, first, 3, 10, 20);
    for (int t = 0; t < 1500 && s.steering.index < 1; t += 10) {
        _sim_run(&s, 10);
    }
    CHECK(s.steering.index == 1, "first batch under way, index %d", s.steering.index);
    _path(&s, second, 2, 10, 20);
    CHECK(s.steering.index == 0 && s.steering.completion == DB_STEERING_DONE_IN_PROGRESS, "a new batch restarts at index 0, %d", s.steering.index);
    run_t r = _run_path(&s, second, 2, 2000);
    CHECK(s.steering.state == DB_STEERING_ARRIVED && _miss(&s, 1400, 1100) < 12.0f, "and arrives at its end, state %d, missed by %.1f", s.steering.state, _miss(&s, 1400, 1100));
    CHECK(_indices_count_up(&r, 2), "its index counts up, %d changes", r.n_indices);
}

static void test_pass_beyond_the_leg(void) {
    // Outside the pass radius but past the point along the leg: passed, not orbited
    db_steering_t        steering;
    db_steering_output_t out;
    db_steering_path_t   path = { .count = 2, .threshold_mm = 10, .pass_mm = 5, .points = { { .x_mm = 1000, .y_mm = 1000 }, { .x_mm = 1500, .y_mm = 1000 } } };
    db_steering_pose_t   pose = { .status = DB_STEERING_POSE_TRACKING, .x_mm = 1000, .y_mm = 500, .heading_deg = 0 };
    db_steering_init(&steering, &_conf);
    db_steering_set_path(&steering, &path);
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.index == 0, "starting: index 0, %d", steering.index);
    pose.x_mm = 1040;
    pose.y_mm = 995;
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.index == 0, "40 mm to the side and short of it: not passed, index %d", steering.index);
    pose.y_mm = 1010;
    db_steering_step(&steering, &pose, 10, &out);
    CHECK(steering.index == 1, "40 mm to the side and past it: passed, index %d", steering.index);
}

static void test_pose_in_the_middle(void) {
    // Stop at the first point facing +x (-90), then go on to the second
    db_steering_path_t path = { .count = 2, .threshold_mm = 10, .points = { { .x_mm = 1000, .y_mm = 800, .has_heading = true, .heading_deg = -90 }, { .x_mm = 1300, .y_mm = 800 } } };
    sim_t              s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    db_steering_set_path(&s.steering, &path);
    int   turned             = 0;
    float at_advance_heading = 999, at_advance_miss = 999;
    for (int t = 0; t < 2000 && db_steering_active(&s.steering); t += 10) {
        float heading = _true_heading_deg(&s), miss = _miss(&s, 1000, 800);
        _sim_run(&s, 10);
        turned |= s.steering.state == DB_STEERING_FINAL_TURN && s.steering.index == 0;
        if (turned && s.steering.index == 1 && at_advance_miss > 998) {
            at_advance_heading = heading;  // as it stood when it went on
            at_advance_miss    = miss;
        }
    }
    _sim_run(&s, 50);
    CHECK(turned, "pose mid-batch: turns in place there");
    CHECK(fabsf(_angle_diff(at_advance_heading, -90)) < 5.0f && at_advance_miss < 12.0f, "and goes on facing it, heading %.1f, axle %.1f mm off", at_advance_heading, at_advance_miss);
    CHECK(s.steering.state == DB_STEERING_ARRIVED && _miss(&s, 1300, 800) < 12.0f, "then arrives at the last point, state %d, missed by %.1f", s.steering.state, _miss(&s, 1300, 800));
}

static void test_max_speed(void) {
    const float pts[][2] = { { 1000, 1300 } };
    sim_t       s;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    db_steering_set_max_speed(&s.steering, 150.0f);
    _path(&s, pts, 1, 10, 0);
    _sim_until_done(&s, 2000);
    CHECK(s.steering.state == DB_STEERING_ARRIVED && s.max_v <= 150.0f + 0.5f && s.max_v > 140.0f, "a 150 mm/s limit holds, fastest %.0f", s.max_v);
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    db_steering_set_max_speed(&s.steering, 450.0f);
    db_steering_set_max_speed(&s.steering, 0);
    _path(&s, pts, 1, 10, 0);
    _sim_until_done(&s, 2000);
    CHECK(fabsf(s.max_v - DB_STEERING_V_MAX_MM_S) < 1.0f, "0 restores the default, fastest %.0f", s.max_v);
}

static void test_precise_arrival(void) {
    const float thresholds[] = { 3, 2, 1 };
    const float bearings[]   = { 0, 30, -150 };
    for (unsigned b = 0; b < 3; b++) {
        for (unsigned i = 0; i < 3; i++) {
            sim_t s;
            float x, y;
            _sim_init(&s, 1000, 500, 0, 1, 0.04f);
            s.robot.noise_mm = 0.3f;
            _target_from(300, bearings[b], &x, &y);
            _goto(&s, x, y, thresholds[i]);
            _sim_until_done(&s, 3000);
            CHECK(s.steering.state == DB_STEERING_ARRIVED, "precise %.0f mm at %.0f: arrived, state %d fail %d, %u corrections", thresholds[i], bearings[b], s.steering.state, s.steering.fail, s.steering.nudges);
            CHECK(_miss(&s, x, y) <= thresholds[i] + 0.5f, "precise %.0f mm at %.0f: true axle within the threshold, missed by %.2f", thresholds[i], bearings[b], _miss(&s, x, y));
            CHECK(s.settles >= 1 && s.steering.nudges <= DB_STEERING_SETTLE_NUDGES, "precise %.0f mm at %.0f: settled, %d settles, %u corrections", thresholds[i], bearings[b], s.settles, s.steering.nudges);
        }
    }
}

static void test_precise_creep_stops_on_time(void) {
    // The poll stops the creep between steps, where a step is 2 mm of it
    const float distances[] = { 300, 305, 311, 317 };
    float       worst       = 0;
    for (unsigned i = 0; i < 4; i++) {
        sim_t s;
        float x, y;
        _sim_init(&s, 1000, 500, 0, 1, 0.04f);
        s.robot.noise_mm = 0.3f;
        _target_from(distances[i], 0, &x, &y);
        _goto(&s, x, y, 2);
        _sim_until_done(&s, 3000);
        worst = fmaxf(worst, s.first_settle_miss);
    }
    CHECK(worst >= 0 && worst < 2.0f, "the creep's first stop lands within 2 mm along the heading, worst %.2f", worst);
}

static void test_precise_fails_to_settle(void) {
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.04f);
    s.robot.noise_mm = 0.3f;
    _target_from(300, 0, &x, &y);
    _goto(&s, x, y, 1);
    for (int t = 0; t < 2000 && s.steering.state != DB_STEERING_SETTLE; t++) {
        _sim_run(&s, 1);
    }
    s.robot.blocked = 1;  // every correction then goes nowhere
    // push the goal out of reach so the first check fails
    s.steering.target.y_mm += 5.0f;
    _sim_until_done(&s, 6000);
    CHECK(s.steering.state == DB_STEERING_FAILED && s.steering.fail == DB_STEERING_FAIL_SETTLE && s.steering.completion == DB_STEERING_DONE_FAILED,
          "corrections that go nowhere: FAILED to settle, state %d fail %d", s.steering.state, s.steering.fail);
    CHECK(s.steering.nudges == DB_STEERING_SETTLE_NUDGES, "after the allowed corrections, %u", s.steering.nudges);
}

static void test_path_from_wire(void) {
    uint8_t            buf[1 + 2 + 1 + 16 * 8 + 4 + 16 * 2 + 8] = { 0 };
    db_steering_path_t path;
    uint8_t            batch;
    size_t             n = 0;
    // threshold 7, 2 points, no trailer
    buf[n++]       = 7;
    buf[n++]       = 0;
    buf[n++]       = 2;
    uint32_t pt[4] = { 1000, 2000, 1500, 2500 };
    memcpy(&buf[n], pt, sizeof(pt));
    n += sizeof(pt);
    CHECK(db_steering_path_from_wire(buf, n, &path, &batch) && path.count == 2 && batch == 0 && path.threshold_mm == 7.0f, "points only: 2 points, no batch id");
    CHECK(path.points[1].x_mm == 1500.0f && path.points[1].y_mm == 2500.0f && !path.points[0].has_heading && !path.points[1].has_heading, "points only: coordinates, no headings");
    CHECK(!db_steering_path_from_wire(buf, n - 1, &path, &batch), "a point cut short is refused");

    // trailer: batch 9, tolerance 4, pass 30, headings none and -90.00
    uint8_t trailer[4] = { 9, 4, 30, 0 };
    memcpy(&buf[n], trailer, sizeof(trailer));
    n += sizeof(trailer);
    int16_t headings[2] = { DB_WAYPOINT_NO_HEADING, -9000 };
    memcpy(&buf[n], headings, sizeof(headings));
    n += sizeof(headings);
    CHECK(db_steering_path_from_wire(buf, n, &path, &batch) && batch == 9 && path.heading_tol_deg == 4.0f && path.pass_mm == 30.0f, "trailer: batch id, tolerance, pass radius");
    CHECK(!path.points[0].has_heading && path.points[1].has_heading && path.points[1].heading_deg == -90.0f, "trailer: headings, none and -90");
    CHECK(db_steering_path_from_wire(buf, n - 1, &path, &batch) && batch == 0 && !path.points[1].has_heading, "a trailer cut short is ignored");

    // more points than a batch holds: the first DB_STEERING_MAX_POINTS kept, the trailer still read
    uint8_t  big[3 + 17 * 8 + 4 + 17 * 2] = { 10, 0, 17 };
    uint32_t xy[2]                        = { 100, 200 };
    for (int i = 0; i < 17; i++) {
        xy[0] = 100U + (uint32_t)i;
        memcpy(&big[3 + i * 8], xy, sizeof(xy));
    }
    uint8_t big_trailer[4] = { 42, 0, 0, 0 };
    memcpy(&big[3 + 17 * 8], big_trailer, sizeof(big_trailer));
    for (int i = 0; i < 17; i++) {
        int16_t h = (int16_t)(100 * i);
        memcpy(&big[3 + 17 * 8 + 4 + i * 2], &h, sizeof(h));
    }
    CHECK(db_steering_path_from_wire(big, sizeof(big), &path, &batch) && path.count == DB_STEERING_MAX_POINTS && batch == 42, "17 points: 16 kept, trailer read, count %d batch %d", path.count, batch);
    CHECK(path.points[15].x_mm == 115.0f && path.points[15].has_heading && path.points[15].heading_deg == 15.0f, "17 points: the 16th point and its heading, %.0f %.2f", path.points[15].x_mm, path.points[15].heading_deg);
    CHECK(!db_steering_path_from_wire(big, 3 + 16 * 8, &path, &batch), "17 points declared, 16 sent: refused");

    // an empty batch is a stop
    uint8_t stop[4] = { 10, 0, 0, 5 };
    CHECK(db_steering_path_from_wire(stop, sizeof(stop), &path, &batch) && path.count == 0, "count 0: a stop");
    CHECK(!db_steering_path_from_wire(stop, 2, &path, &batch), "no count: refused");
}

static void test_final_heading_unbiased(void) {
    // A turn in place ends near the centre of the tolerance band, not at the edge it
    // enters from: the mean signed error over turns both ways stays small
    const float headings[] = { 90, -150, 45, -90, 135, -30, 170, -170 };
    float       sum = 0, worst = 0;
    for (unsigned i = 0; i < 8; i++) {
        sim_t s;
        _sim_init(&s, 1000, 500, 0, 1, 0.04f);
        s.robot.noise_mm        = 0.3f;
        db_steering_path_t path = { .count = 1, .threshold_mm = 10, .points = { { .x_mm = 1000, .y_mm = 500, .has_heading = true, .heading_deg = headings[i] } } };
        db_steering_set_path(&s.steering, &path);
        _sim_until_done(&s, 1500);
        float e = _angle_diff(_true_heading_deg(&s), headings[i]);
        // the sign that says which side of the band it stopped on, relative to the turn
        float turned = _angle_diff(headings[i], 0);
        sum += (turned > 0) ? -e : e;
        worst = fmaxf(worst, fabsf(e));
    }
    CHECK(fabsf(sum / 8.0f) < 0.6f, "final turns stop short on average by %.2f deg, worst %.1f", sum / 8.0f, worst);
}

int main(void) {
    test_idle_holds_still();
    test_target_ahead();
    test_no_overshoot();
    test_target_behind_pivots();
    test_target_to_the_side();
    test_very_close_targets();
    test_near_axle_target_no_pivot();
    test_overshoot_reverses();
    test_already_there();
    test_arrival_latches();
    test_final_heading();
    test_no_heading_spins_then_drives();
    test_no_heading_times_out();
    test_lost_holds_then_resumes();
    test_lost_for_good_fails();
    test_heading_lost_mid_move_recovers_straight();
    test_recover_straight_closed_loop();
    test_new_target_mid_drive();
    test_retarget_after_arrival();
    test_speed_falls_with_heading_error();
    test_arrival_counts_the_runon();
    test_turn_is_predicted();
    test_blocked_wheels_fail_on_progress();
    test_turn_never_exceeds_spin_limit();
    test_pivot_without_floor();
    test_stop_goes_idle();
    test_square();
    test_zigzag();
    test_gentle_curve_keeps_speed();
    test_sharp_corner_slows();
    test_u_turn();
    test_point_behind();
    test_completion();
    test_stop_keeps_the_outcome();
    test_empty_path_stops();
    test_failed_mid_path();
    test_retarget_mid_path();
    test_pass_beyond_the_leg();
    test_pose_in_the_middle();
    test_max_speed();
    test_precise_arrival();
    test_precise_creep_stops_on_time();
    test_precise_fails_to_settle();
    test_path_from_wire();
    test_final_heading_unbiased();
    printf("%d passed, %d failed\n", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
