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

#include "geometry.h"
#include "pose_estimator.h"
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
    .lever_mm                   = DB_LH2_LEVER_ARM_EFFECTIVE,
    .lever_angle_deg            = DB_LH2_LEVER_ANGLE,
    .r_pos_mm2                  = DB_POSE_ESTIMATOR_R_POS_MM2,
    .q_pos_mm2_per_mm           = DB_POSE_ESTIMATOR_Q_POS_MM2_PER_MM,
    .q_heading_roll_deg2_per_mm = DB_POSE_ESTIMATOR_Q_HEADING_ROLL_DEG2_PER_MM,
    .q_heading_turn_deg2_per_mm = DB_POSE_ESTIMATOR_Q_HEADING_TURN_DEG2_PER_MM,
    .turn_speed_ref_mm_s        = DB_POSE_ESTIMATOR_TURN_SPEED_REF_MM_S,
    .gate                       = DB_POSE_ESTIMATOR_GATE,
    .fix_age_ticks              = DB_POSE_ESTIMATOR_FIX_AGE_TICKS,
    .timeout_ticks              = DB_POSE_ESTIMATOR_TIMEOUT_TICKS,
    .seed_fixes                 = DB_POSE_ESTIMATOR_SEED_FIXES,
    .seed_tolerance_mm          = DB_POSE_ESTIMATOR_SEED_TOLERANCE_MM,
    .acquire_mm                 = DB_POSE_ESTIMATOR_ACQUIRE_MM,
    .kidnap_fixes               = DB_POSE_ESTIMATOR_KIDNAP_FIXES,
    .kidnap_still_mm            = DB_POSE_ESTIMATOR_KIDNAP_STILL_MM,
};

static const db_steering_conf_t _conf = {
    .lever_mm         = DB_LH2_LEVER_ARM_EFFECTIVE,
    .v_max_mm_s       = DB_STEERING_V_MAX_MM_S,
    .approach_per_s   = DB_STEERING_APPROACH_PER_S,
    .runon_s          = DB_STEERING_RUNON_S,
    .spin_mm_s        = DB_STEERING_SPIN_MM_S,
    .spin_min_mm_s    = DB_STEERING_SPIN_MIN_MM_S,
    .heading_kp       = DB_STEERING_HEADING_KP,
    .heading_kd       = DB_STEERING_HEADING_KD,
    .align_enter_deg  = DB_STEERING_ALIGN_ENTER_DEG,
    .align_exit_deg   = DB_STEERING_ALIGN_EXIT_DEG,
    .full_speed_deg   = DB_STEERING_FULL_SPEED_DEG,
    .final_tol_deg    = DB_STEERING_FINAL_TOL_DEG,
    .near_mm          = DB_STEERING_NEAR_MM,
    .bearing_min_mm   = DB_STEERING_BEARING_MIN_MM,
    .lookahead_s      = DB_STEERING_LOOKAHEAD_S,
    .arrival_min_mm   = DB_STEERING_ARRIVAL_MIN_MM,
    .no_heading_ticks = DB_STEERING_NO_HEADING_TICKS,
    .turn_ticks       = DB_STEERING_TURN_TICKS,
    .progress_ticks   = DB_STEERING_PROGRESS_TICKS,
    .progress_mm      = DB_STEERING_PROGRESS_MM,
    .hold_ticks       = DB_STEERING_HOLD_TICKS,
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
} robot_t;

typedef struct {
    robot_t              robot;
    db_pose_estimator_t  est;
    db_steering_t        steering;
    db_steering_output_t out;
    float                max_diff;      ///< largest |left - right| / 2 ever commanded, mm/s
    float                max_wheel;     ///< largest |wheel| ever commanded, mm/s
    uint32_t             arrived_tick;  ///< tick ARRIVED was first seen, 0 before
    int                  pivot_steps;   ///< outer steps commanded in place while turning
    int                  steps;         ///< outer steps
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

static void _true_photodiode(const sim_t *s, float *px, float *py) {
    _photodiode(s->robot.x, s->robot.y, s->robot.theta, px, py);
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
    *s             = (sim_t){ 0 };
    s->robot.x     = x;
    s->robot.y     = y;
    s->robot.theta = heading_deg * DEG;
    s->robot.tau_s = tau_s;
    s->robot.seed  = 12345U;
    s->robot.fixes = 1;
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
/// steering every DB_STEERING_PERIOD_TICKS
static void _sim_run(sim_t *s, uint32_t ticks) {
    for (uint32_t t = 0; t < ticks; t++) {
        robot_t *r = &s->robot;
        if (r->ticks % DB_STEERING_PERIOD_TICKS == 0) {
            db_steering_pose_t pose;
            _pose_of(&s->est, &pose);
            db_steering_step(&s->steering, &pose, DB_STEERING_PERIOD_TICKS, &s->out);
            s->steps++;
            if (!s->out.brake) {
                float diff   = fabsf(s->out.left_mm_s - s->out.right_mm_s) / 2.0f;
                s->max_diff  = fmaxf(s->max_diff, diff);
                s->max_wheel = fmaxf(s->max_wheel, fmaxf(fabsf(s->out.left_mm_s), fabsf(s->out.right_mm_s)));
                if (diff > 1.0f && fabsf(s->out.left_mm_s + s->out.right_mm_s) < 1e-3f) {
                    s->pivot_steps++;
                }
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
            db_pose_estimator_update(&s->est, zx + _noise(r), zy + _noise(r));
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

static float _miss(const sim_t *s, float x, float y) {
    float px, py;
    _true_photodiode(s, &px, &py);
    return hypotf(px - x, py - y);
}

/// The robot faces +y from (1000, 500); a target at distance and bearing from its photodiode
static void _target_from(float distance, float bearing_deg, float *x, float *y) {
    float px, py;
    _photodiode(1000.0f, 500.0f, 0, &px, &py);
    *x = px - distance * sinf(bearing_deg * DEG);
    *y = py + distance * cosf(bearing_deg * DEG);
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
    // Photodiode along the line of approach, tick by tick
    sim_t s;
    float x, y;
    _sim_init(&s, 1000, 500, 0, 1, 0.05f);
    _target_from(800, 0, &x, &y);
    _goto(&s, x, y, 5);
    float past = -INFINITY;
    for (int t = 0; t < 1200; t++) {
        _sim_run(&s, 1);
        float px, py;
        _true_photodiode(&s, &px, &py);
        past = fmaxf(past, py - y);
    }
    CHECK(s.steering.state == DB_STEERING_ARRIVED, "800 mm at 300 mm/s: arrived, state %d", s.steering.state);
    CHECK(past < 5.0f, "the photodiode never passes the target by 5 mm, passed by %.1f", past);
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
    // 25 mm ahead of the photodiode, and 25, 60 and 100 mm behind it
    const float distances[] = { 25, 25, 60, 100 };
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

static void test_overshoot_reverses(void) {
    // Driving at 200 mm/s with the target already 30 mm behind the photodiode
    const float behind[] = { 30, 100 };
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
        CHECK(_miss(&s, x, y) < 14.0f, "final heading %.0f: photodiode within 14 mm after the turn, missed by %.1f", headings[i], _miss(&s, x, y));
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
    CHECK(spun < 150, "no heading: the estimator tracks within 1.5 s of spinning, took %d ticks", spun);
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

static void test_heading_lost_mid_move_fails(void) {
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
    CHECK(steering.state == DB_STEERING_FAILED && steering.fail == DB_STEERING_FAIL_HEADING_LOST, "kidnap mid-move: FAILED, state %d fail %d", steering.state, steering.fail);
    CHECK(out.brake && out.left_mm_s == 0 && out.right_mm_s == 0, "and brakes");
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
    // At cruise the robot runs on 15 mm, so a photodiode 30 mm short of a
    // 20 mm threshold is not arrived and one 10 mm short is
    db_steering_t        steering;
    db_steering_output_t out;
    _cruising(&steering, &out);
    float              axle_y = 1000.0f - DB_LH2_LEVER_ARM_EFFECTIVE - 20.0f - 30.0f;
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
    target.x_mm              = 1000.0f - DB_LH2_LEVER_ARM_EFFECTIVE * sinf(4.0f * DEG);
    target.y_mm              = 500.0f + DB_LH2_LEVER_ARM_EFFECTIVE * cosf(4.0f * DEG);
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

int main(void) {
    test_idle_holds_still();
    test_target_ahead();
    test_no_overshoot();
    test_target_behind_pivots();
    test_target_to_the_side();
    test_very_close_targets();
    test_overshoot_reverses();
    test_already_there();
    test_arrival_latches();
    test_final_heading();
    test_no_heading_spins_then_drives();
    test_no_heading_times_out();
    test_lost_holds_then_resumes();
    test_lost_for_good_fails();
    test_heading_lost_mid_move_fails();
    test_new_target_mid_drive();
    test_retarget_after_arrival();
    test_speed_falls_with_heading_error();
    test_arrival_counts_the_runon();
    test_turn_is_predicted();
    test_blocked_wheels_fail_on_progress();
    test_turn_never_exceeds_spin_limit();
    test_pivot_without_floor();
    test_stop_goes_idle();
    printf("%d passed, %d failed\n", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
