/**
 * @file
 * @brief  Host test of drv/pose_estimator against a simulated robot
 *
 * The simulated robot turns over the same effective track the estimator uses, and its
 * photodiode sits on the same lever arm, so these checks prove the filter's
 * logic, not the model's constants.
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "geometry.h"
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

#define DEG ((float)M_PI / 180.0f)

static const db_pose_estimator_conf_t _conf = {
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

//=========================== simulated robot ==================================

#define SIM_SUBSTEPS    (20)
#define TICKS_PER_FIX   (10U)
#define LH2_NOISE_SD_MM (1.0f)

#define SIM_HISTORY (16U)

typedef struct {
    float x;      ///< axle midpoint, mm
    float y;      ///< axle midpoint, mm
    float theta;  ///< rad, 0 along +y, clockwise positive
} pose_t;

typedef struct {
    float    x;                     ///< axle midpoint, mm
    float    y;                     ///< axle midpoint, mm
    float    theta;                 ///< rad, 0 along +y, clockwise positive
    uint32_t seed;                  ///< noise generator state
    uint32_t steps;                 ///< steps taken
    pose_t   history[SIM_HISTORY];  ///< pose after each recent step
    uint32_t fix_age;               ///< steps a fix lags the robot by
} robot_t;

static void _robot_step(robot_t *r, int32_t counts_left, int32_t counts_right) {
    float dl = (float)counts_left * DB_MM_PER_COUNT / SIM_SUBSTEPS;
    float dr = (float)counts_right * DB_MM_PER_COUNT / SIM_SUBSTEPS;
    for (int i = 0; i < SIM_SUBSTEPS; i++) {
        float d  = 0.5f * (dl + dr);
        float dt = -(dr - dl) / db_track_effective_mm(dl, dr);
        float m  = r->theta + 0.5f * dt;
        r->x += -d * sinf(m);
        r->y += d * cosf(m);
        r->theta += dt;
    }
    r->history[r->steps % SIM_HISTORY] = (pose_t){ r->x, r->y, r->theta };
    r->steps++;
}

static float _noise(robot_t *r) {
    // Box-Muller on a 32-bit LCG: reproducible across hosts
    r->seed  = r->seed * 1664525U + 1013904223U;
    float u1 = ((float)(r->seed >> 8) + 1.0f) / 16777217.0f;
    r->seed  = r->seed * 1664525U + 1013904223U;
    float u2 = (float)(r->seed >> 8) / 16777216.0f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

/// The photodiode as LH2 reports it: the pose fix_age steps ago, or now
/// before the robot has taken that many
static void _robot_sensor(robot_t *r, float noise_sd, float *x, float *y) {
    pose_t p = { r->x, r->y, r->theta };
    if (r->fix_age > 0 && r->steps > r->fix_age) {
        p = r->history[(r->steps - 1 - r->fix_age) % SIM_HISTORY];
    }
    *x = p.x - _conf.lever_mm * sinf(p.theta) + noise_sd * _noise(r);
    *y = p.y + _conf.lever_mm * cosf(p.theta) + noise_sd * _noise(r);
}

static float _angle_error_deg(float a_rad, float b_rad) {
    float e = fmodf(a_rad - b_rad, 2.0f * (float)M_PI);
    if (e >= (float)M_PI) {
        e -= 2.0f * (float)M_PI;
    } else if (e < -(float)M_PI) {
        e += 2.0f * (float)M_PI;
    }
    return fabsf(e) / DEG;
}

/// Drive both for a number of ticks, with a fix every TICKS_PER_FIX when fixes is set
static uint32_t _run(db_pose_estimator_t *est, robot_t *r, int32_t cl, int32_t cr, uint32_t ticks, int fixes, float noise_sd) {
    uint32_t accepted = 0;
    for (uint32_t t = 1; t <= ticks; t++) {
        _robot_step(r, cl, cr);
        db_pose_estimator_predict(est, cl, cr, 1);
        if (fixes && (t % TICKS_PER_FIX) == 0) {
            float zx, zy;
            _robot_sensor(r, noise_sd, &zx, &zy);
            if (db_pose_estimator_update(est, zx, zy) == DB_POSE_ESTIMATOR_ACCEPTED) {
                accepted++;
            }
        }
    }
    return accepted;
}

//=========================== tests ============================================

static void test_straight_prediction(void) {
    db_pose_estimator_t est;
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, 1000, 1000, 0, 1);
    for (int t = 0; t < 100; t++) {
        db_pose_estimator_predict(&est, 50, 50, 1);
    }
    float d = 5000 * DB_MM_PER_COUNT;
    CHECK(fabsf(est.x - 1000) < 1e-3f && fabsf(est.y - (1000 + d)) < 0.05f, "heading 0 drives along +y: got (%.2f, %.2f), want (1000, %.2f)", est.x, est.y, 1000 + d);

    db_pose_estimator_seed(&est, 0, 0, -90, 1);
    for (int t = 0; t < 100; t++) {
        db_pose_estimator_predict(&est, 50, 50, 1);
    }
    float h = 0;
    db_pose_estimator_heading_deg(&est, &h);
    CHECK(fabsf(est.x - d) < 0.05f && fabsf(est.y) < 0.05f && fabsf(h + 90) < 1e-3f, "heading -90 drives along +x: got (%.2f, %.2f) at %.2f deg", est.x, est.y, h);
}

static void test_arc_prediction(void) {
    db_pose_estimator_t est;
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, 0, 0, 0, 1);
    // Left faster, so the turn is clockwise and heading rises
    for (int t = 0; t < 200; t++) {
        db_pose_estimator_predict(&est, 20, 10, 1);
    }
    float distance = 200 * 15 * DB_MM_PER_COUNT;
    float turn     = 200 * 10 * DB_MM_PER_COUNT / DB_TRACK_EFFECTIVE_ARC;
    float k        = turn / distance;
    float want_x   = (cosf(turn) - 1.0f) / k;
    float want_y   = sinf(turn) / k;
    CHECK(fabsf(est.x - want_x) < 0.1f && fabsf(est.y - want_y) < 0.1f, "arc end: got (%.2f, %.2f), want (%.2f, %.2f)", est.x, est.y, want_x, want_y);
    CHECK(fabsf(est.theta - turn) < 1e-4f, "arc turn: got %.4f rad, want %.4f", est.theta, turn);
}

static void test_spin_prediction(void) {
    db_pose_estimator_t est;
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, 500, 500, 0, 1);
    for (int t = 0; t < 100; t++) {
        db_pose_estimator_predict(&est, 5, -5, 1);
    }
    float turn = 100 * 10 * DB_MM_PER_COUNT / DB_TRACK_EFFECTIVE;
    CHECK(fabsf(est.x - 500) < 1e-3f && fabsf(est.y - 500) < 1e-3f, "a spin leaves the axle in place: got (%.3f, %.3f)", est.x, est.y);
    CHECK(fabsf(est.theta - turn) < 1e-4f, "a spin turns by the travel difference over the effective track: got %.4f, want %.4f", est.theta, turn);
}

static void test_effective_track(void) {
    CHECK(db_track_effective_mm(100, -100) == DB_TRACK_EFFECTIVE, "a spin uses the spin track, got %.2f", db_track_effective_mm(100, -100));
    CHECK(db_track_effective_mm(-100, 100) == DB_TRACK_EFFECTIVE, "either way round, got %.2f", db_track_effective_mm(-100, 100));
    float pivot = DB_TRACK_EFFECTIVE + (DB_TRACK_EFFECTIVE_ARC - DB_TRACK_EFFECTIVE) / DB_TRACK_EFFECTIVE_ARC_RATIO;
    CHECK(fabsf(db_track_effective_mm(0, 100) - pivot) < 1e-3f && fabsf(db_track_effective_mm(0, -100) - pivot) < 1e-3f, "a pivot on one wheel is part way, got %.2f, want %.2f", db_track_effective_mm(0, 100), pivot);
    CHECK(fabsf(db_track_effective_mm(0, 10) - db_track_effective_mm(0, 1000)) < 1e-3f, "only the ratio counts");
    float r100 = 0.5f * DB_TRACK_EFFECTIVE_ARC_RATIO;
    CHECK(fabsf(db_track_effective_mm(r100 + 0.5f, r100 - 0.5f) - DB_TRACK_EFFECTIVE_ARC) < 1e-3f, "a 100 mm radius reaches the arc track, got %.2f", db_track_effective_mm(r100 + 0.5f, r100 - 0.5f));
    CHECK(db_track_effective_mm(300, 200) == DB_TRACK_EFFECTIVE_ARC && db_track_effective_mm(-300, -200) == DB_TRACK_EFFECTIVE_ARC, "wider arcs hold the arc track, forward or back");
    CHECK(db_track_effective_mm(100, 100) == DB_TRACK_EFFECTIVE_ARC && db_track_effective_mm(0, 0) == DB_TRACK_EFFECTIVE_ARC, "no turn does not divide by zero");
}

static void test_lever_arm_sensor(void) {
    db_pose_estimator_t est;
    db_pose_estimator_init(&est, &_conf);
    float x = 0, y = 0;
    CHECK(!db_pose_estimator_sensor(&est, &x, &y), "no photodiode estimate before a pose");
    db_pose_estimator_seed(&est, 100, 200, 0, 1);
    db_pose_estimator_sensor(&est, &x, &y);
    CHECK(fabsf(x - 100) < 1e-3f && fabsf(y - (200 + _conf.lever_mm)) < 1e-3f, "heading 0 puts the photodiode along +y: got (%.2f, %.2f)", x, y);
    db_pose_estimator_seed(&est, 100, 200, 90, 1);
    db_pose_estimator_sensor(&est, &x, &y);
    CHECK(fabsf(x - (100 - _conf.lever_mm)) < 1e-3f && fabsf(y - 200) < 1e-3f, "heading 90 puts the photodiode along -x: got (%.2f, %.2f)", x, y);
}

static void test_lever_arm_spin_update(void) {
    // Turning in place swings the photodiode round a 51.5 mm circle; with the
    // lever arm in H every fix fits, and without it the fixes are outliers
    db_pose_estimator_t est;
    robot_t             r = { .x = 1500, .y = 1000, .theta = 30 * DEG, .seed = 1, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 30, 2);
    uint32_t accepted = _run(&est, &r, 5, -5, 700, 1, LH2_NOISE_SD_MM);
    CHECK(accepted == 70, "all 70 fixes of a spin fit the lever arm, got %u", accepted);
    CHECK(_angle_error_deg(est.theta, r.theta) < 1.0f, "heading tracks the spin, %.2f deg off", _angle_error_deg(est.theta, r.theta));
    CHECK(hypotf(est.x - r.x, est.y - r.y) < 2.0f, "the axle stays put, %.2f mm off", hypotf(est.x - r.x, est.y - r.y));
    CHECK(est.predicts > est.accepted, "predict runs more often than update: %u against %u", est.predicts, est.accepted);

    db_pose_estimator_conf_t no_lever = _conf;
    no_lever.lever_mm                 = 0;
    robot_t r2                        = { .x = 1500, .y = 1000, .theta = 30 * DEG, .seed = 1, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &no_lever);
    db_pose_estimator_seed(&est, r2.x, r2.y, 30, 2);
    accepted = _run(&est, &r2, 5, -5, 300, 1, LH2_NOISE_SD_MM);
    CHECK(accepted < 30, "without the lever arm a spin's fixes do not fit, %u of 30 accepted", accepted);
}

static void test_converges_from_wrong_heading(void) {
    db_pose_estimator_t est;
    robot_t             r = { .x = 1000, .y = 1000, .theta = 40 * DEG, .seed = 2, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 0, 60);
    _run(&est, &r, 10, 10, 300, 1, LH2_NOISE_SD_MM);
    CHECK(_angle_error_deg(est.theta, r.theta) < 3.0f, "40 deg of seed error is gone after 3 s straight, %.2f deg left", _angle_error_deg(est.theta, r.theta));
    CHECK(est.rejected == 0, "no fix is rejected while converging, got %u", est.rejected);
}

static void test_acquire_heading_from_motion(void) {
    db_pose_estimator_t est;
    robot_t             r = { .x = 2000, .y = 800, .theta = 130 * DEG, .seed = 3, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    _run(&est, &r, 0, 0, 100, 1, LH2_NOISE_SD_MM);
    float h = 0;
    CHECK(!db_pose_estimator_heading_deg(&est, &h), "no heading from a robot that has not moved");
    CHECK(est.status == DB_POSE_ESTIMATOR_SEEDING, "still seeding at rest, status %d", est.status);

    _run(&est, &r, 10, 10, 100, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING, "tracking after 1 s straight, status %d", est.status);
    CHECK(_angle_error_deg(est.theta, r.theta) < 5.0f, "acquired heading within 5 deg, %.2f off", _angle_error_deg(est.theta, r.theta));
    _run(&est, &r, 10, 10, 200, 1, LH2_NOISE_SD_MM);
    CHECK(_angle_error_deg(est.theta, r.theta) < 2.0f, "within 2 deg after 2 s more, %.2f off", _angle_error_deg(est.theta, r.theta));
    CHECK(hypotf(est.x - r.x, est.y - r.y) < 3.0f, "axle within 3 mm, %.2f off", hypotf(est.x - r.x, est.y - r.y));
}

static void test_acquire_heading_from_spin(void) {
    // No translation at all: only the lever arm can reveal the heading
    db_pose_estimator_t est;
    robot_t             r = { .x = 1200, .y = 1200, .theta = -60 * DEG, .seed = 4, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    _run(&est, &r, -5, 5, 300, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING, "a spin in place acquires heading, status %d", est.status);
    CHECK(_angle_error_deg(est.theta, r.theta) < 5.0f, "heading from a spin within 5 deg, %.2f off", _angle_error_deg(est.theta, r.theta));
    CHECK(hypotf(est.x - r.x, est.y - r.y) < 3.0f, "axle from a spin within 3 mm, %.2f off", hypotf(est.x - r.x, est.y - r.y));
}

static void test_gate_rejects_outlier(void) {
    db_pose_estimator_t est;
    robot_t             r = { .x = 1000, .y = 1000, .theta = 0, .seed = 5, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 0, 2);
    _run(&est, &r, 0, 0, 100, 1, LH2_NOISE_SD_MM);
    float x = est.x, y = est.y, theta = est.theta;
    float zx, zy;
    _robot_sensor(&r, 0, &zx, &zy);
    CHECK(db_pose_estimator_update(&est, zx + 300, zy) == DB_POSE_ESTIMATOR_REJECTED, "a fix 300 mm off is rejected, d2 %.1f", est.last_d2);
    CHECK(est.x == x && est.y == y && est.theta == theta, "a rejected fix leaves the pose alone");
    CHECK(db_pose_estimator_update(&est, zx, zy) == DB_POSE_ESTIMATOR_ACCEPTED, "the next good fix is accepted, d2 %.1f", est.last_d2);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING, "one outlier does not lose the pose, status %d", est.status);
}

static void test_kidnap_reseeds(void) {
    // Picked up and put down elsewhere, turned, wheels still: the estimator
    // gives up the pose after DB_POSE_ESTIMATOR_KIDNAP_FIXES fixes, not the timeout
    db_pose_estimator_t est;
    robot_t             r = { .x = 1000, .y = 1000, .theta = 0, .seed = 6, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 0, 2);
    _run(&est, &r, 0, 0, 50, 1, LH2_NOISE_SD_MM);
    r.x     = 1500;
    r.y     = 1300;
    r.theta = 100 * DEG;
    _run(&est, &r, 0, 0, (_conf.kidnap_fixes - 1) * TICKS_PER_FIX, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING && est.kidnaps == 0, "still tracking one fix short of a kidnap, status %d", est.status);
    _run(&est, &r, 0, 0, TICKS_PER_FIX, 1, LH2_NOISE_SD_MM);
    float h = 0, sx = 0, sy = 0;
    CHECK(est.status == DB_POSE_ESTIMATOR_SEEDING && est.kidnaps == 1, "a kidnap after %u fixes, status %d kidnaps %u", _conf.kidnap_fixes, est.status, est.kidnaps);
    CHECK(!db_pose_estimator_heading_deg(&est, &h) && !db_pose_estimator_sensor(&est, &sx, &sy), "no heading and no pose after a kidnap");
    CHECK(hypotf(est.chain_x - (r.x - _conf.lever_mm * sinf(r.theta)), est.chain_y - (r.y + _conf.lever_mm * cosf(r.theta))) < 5.0f, "the chain starts on the new fixes");
    _run(&est, &r, 0, 0, 100, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_SEEDING && est.seeds == 0, "standing still does not guess a heading, status %d", est.status);
    _run(&est, &r, 10, 10, 150, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING && est.seeds == 1, "reseeded once the robot moves, status %d, seeds %u", est.status, est.seeds);
    CHECK(_angle_error_deg(est.theta, r.theta) < 5.0f, "reseeded heading within 5 deg, %.2f off", _angle_error_deg(est.theta, r.theta));
    CHECK(hypotf(est.x - r.x, est.y - r.y) < 5.0f, "reseeded axle within 5 mm, %.2f off", hypotf(est.x - r.x, est.y - r.y));
}

static void test_outlier_still_no_kidnap(void) {
    db_pose_estimator_t est;
    robot_t             r = { .x = 1000, .y = 1000, .theta = 0, .seed = 9, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 0, 2);
    _run(&est, &r, 0, 0, 50, 1, LH2_NOISE_SD_MM);
    float zx, zy;
    _robot_sensor(&r, 0, &zx, &zy);
    // Two outliers at the same spot, one short of a kidnap, then a good fix
    for (uint32_t i = 0; i + 1 < _conf.kidnap_fixes; i++) {
        db_pose_estimator_update(&est, zx + 200, zy);
    }
    _run(&est, &r, 0, 0, 10 * TICKS_PER_FIX, 1, LH2_NOISE_SD_MM);
    db_pose_estimator_update(&est, zx + 200, zy);
    _run(&est, &r, 0, 0, 10 * TICKS_PER_FIX, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING && est.kidnaps == 0 && est.seeds == 0, "outliers broken up by good fixes do not reseed, status %d kidnaps %u", est.status, est.kidnaps);
    // Rejected fixes that disagree with each other do not reseed either
    for (uint32_t i = 0; i < 2 * _conf.kidnap_fixes; i++) {
        db_pose_estimator_update(&est, zx + 200 + 50 * (float)i, zy);
    }
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING && est.kidnaps == 0, "scattered rejected fixes do not reseed, status %d kidnaps %u", est.status, est.kidnaps);
}

static void test_rejected_while_driving_times_out(void) {
    // Knocked sideways while driving: the wheels turn, so this is not a
    // kidnap; the pose is held until the timeout, then a chain reseeds
    db_pose_estimator_t est;
    robot_t             r = { .x = 1000, .y = 1000, .theta = 0, .seed = 6, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 0, 2);
    _run(&est, &r, 10, 10, 50, 1, LH2_NOISE_SD_MM);
    r.x += 200;
    _run(&est, &r, 10, 10, _conf.timeout_ticks, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING && est.kidnaps == 0, "still tracking until the timeout, status %d kidnaps %u", est.status, est.kidnaps);
    CHECK(est.rejected >= 9, "fixes after the knock are rejected, %u", est.rejected);
    _run(&est, &r, 10, 10, 20, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_LOST, "lost after the timeout, status %d", est.status);
    _run(&est, &r, 10, 10, 150, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING && est.seeds == 1 && est.kidnaps == 0, "reseeded by the chain, status %d, seeds %u", est.status, est.seeds);
    CHECK(hypotf(est.x - r.x, est.y - r.y) < 5.0f, "reseeded axle within 5 mm, %.2f off", hypotf(est.x - r.x, est.y - r.y));
}

static void test_occlusion_keeps_heading(void) {
    // No fixes at all for 2 s: lost, but odometry carried the pose, so the
    // first fix back is inside the gate and nothing is reseeded
    db_pose_estimator_t est;
    robot_t             r = { .x = 1000, .y = 1000, .theta = 20 * DEG, .seed = 7, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 20, 2);
    _run(&est, &r, 10, 10, 100, 1, LH2_NOISE_SD_MM);
    _run(&est, &r, 10, 10, 200, 0, 0);
    CHECK(est.status == DB_POSE_ESTIMATOR_LOST, "lost after 2 s without a fix, status %d", est.status);
    float zx, zy;
    _robot_sensor(&r, LH2_NOISE_SD_MM, &zx, &zy);
    CHECK(db_pose_estimator_update(&est, zx, zy) == DB_POSE_ESTIMATOR_ACCEPTED, "the first fix back is accepted, d2 %.1f", est.last_d2);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING && est.seeds == 0, "tracking again without a reseed, status %d, seeds %u", est.status, est.seeds);
}

static void test_fix_age_compensated(void) {
    // 300 mm/s straight and a 200 mm/s-per-wheel spin, with every fix 4 ticks
    // old, so leaving the age out is large enough to show
    const uint32_t age  = 4;
    int32_t        fast = (int32_t)lroundf(300.0f * 0.01f / DB_MM_PER_COUNT);
    int32_t        spin = (int32_t)lroundf(200.0f * 0.01f / DB_MM_PER_COUNT);
    for (int aged = 1; aged >= 0; aged--) {
        db_pose_estimator_conf_t conf = _conf;
        conf.fix_age_ticks            = aged ? age : 0;
        db_pose_estimator_t est;
        robot_t             r = { .x = 1000, .y = 1000, .theta = 0, .seed = 8, .fix_age = age };
        db_pose_estimator_init(&est, &conf);
        db_pose_estimator_seed(&est, r.x, r.y, 0, 2);
        _run(&est, &r, fast, fast, 300, 1, LH2_NOISE_SD_MM);
        float straight_mm = hypotf(est.x - r.x, est.y - r.y);
        _run(&est, &r, spin, -spin, 200, 1, LH2_NOISE_SD_MM);
        float spin_deg = _angle_error_deg(est.theta, r.theta);
        if (aged) {
            CHECK(est.rejected == 0, "no fix is rejected at speed once its age is compensated, got %u", est.rejected);
            CHECK(straight_mm < 4.0f, "axle within 4 mm at 300 mm/s, %.2f off", straight_mm);
            CHECK(spin_deg < 2.0f, "heading within 2 deg in a fast spin, %.2f off", spin_deg);
        } else {
            CHECK(est.rejected > 0 || straight_mm > 8.0f, "uncompensated, the same fixes lag or are rejected: %u rejected, %.2f mm off", est.rejected, straight_mm);
        }
    }
}

static void test_noise_scales_with_distance(void) {
    // The distance and turn terms alone: the slip term depends on speed changes
    db_pose_estimator_conf_t conf     = _conf;
    conf.q_pos_slip_mm2_per_mm_s      = 0;
    conf.q_heading_slip_deg2_per_mm_s = 0;
    db_pose_estimator_t a, b;
    db_pose_estimator_init(&a, &conf);
    db_pose_estimator_seed(&a, 0, 0, 0, 2);
    float before = a.P[2][2];
    for (int t = 0; t < 1000; t++) {
        db_pose_estimator_predict(&a, 0, 0, 1);
    }
    CHECK(a.P[0][0] == a.conf->r_pos_mm2 && a.P[2][2] == before, "standing still adds no process noise over 1000 calls");

    // The same spin in 100 calls or in one
    db_pose_estimator_init(&a, &conf);
    db_pose_estimator_init(&b, &conf);
    db_pose_estimator_seed(&a, 0, 0, 0, 2);
    db_pose_estimator_seed(&b, 0, 0, 0, 2);
    for (int t = 0; t < 100; t++) {
        db_pose_estimator_predict(&a, 10, -10, 1);
    }
    db_pose_estimator_predict(&b, 1000, -1000, 100);
    CHECK(fabsf(a.P[2][2] - b.P[2][2]) < 1e-3f * b.P[2][2], "heading noise over a spin is independent of the call rate: %.6g against %.6g", a.P[2][2], b.P[2][2]);

    // The same straight in 100 calls or in one
    db_pose_estimator_init(&a, &conf);
    db_pose_estimator_init(&b, &conf);
    db_pose_estimator_seed(&a, 0, 0, 0, 0);
    db_pose_estimator_seed(&b, 0, 0, 0, 0);
    for (int t = 0; t < 100; t++) {
        db_pose_estimator_predict(&a, 10, 10, 1);
    }
    db_pose_estimator_predict(&b, 1000, 1000, 100);
    CHECK(fabsf(a.P[2][2] - b.P[2][2]) < 1e-3f * b.P[2][2], "heading noise over a straight is independent of the call rate: %.6g against %.6g", a.P[2][2], b.P[2][2]);
    CHECK(fabsf(a.P[1][1] - b.P[1][1]) < 1e-3f * b.P[1][1], "position noise over a straight is independent of the call rate: %.6g against %.6g", a.P[1][1], b.P[1][1]);

    // Twice the distance, twice the heading noise
    db_pose_estimator_init(&b, &conf);
    db_pose_estimator_seed(&b, 0, 0, 0, 0);
    db_pose_estimator_predict(&b, 2000, 2000, 200);
    CHECK(fabsf(b.P[2][2] - 2.0f * a.P[2][2]) < 1e-3f * b.P[2][2], "heading noise doubles with the distance: %.6g against 2 x %.6g", b.P[2][2], a.P[2][2]);
}

static void test_slip_noise_on_hard_stop(void) {
    // Against the same drive with the slip term off: cruising with count
    // jitter adds almost nothing, a stop from 700 mm/s in 100 ms adds a lot
    db_pose_estimator_conf_t off     = _conf;
    off.q_pos_slip_mm2_per_mm_s      = 0;
    off.q_heading_slip_deg2_per_mm_s = 0;
    db_pose_estimator_t est, ref;
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_init(&ref, &off);
    db_pose_estimator_seed(&est, 0, 0, 0, 1);
    db_pose_estimator_seed(&ref, 0, 0, 0, 1);
    int32_t cruise = (int32_t)lroundf(700.0f * 0.01f / DB_MM_PER_COUNT);
    for (int t = 0; t < 30; t++) {
        db_pose_estimator_predict(&est, cruise, cruise, 1);
        db_pose_estimator_predict(&ref, cruise, cruise, 1);
    }
    float h0 = est.P[2][2] - ref.P[2][2];
    for (int t = 0; t < 100; t++) {
        db_pose_estimator_predict(&est, cruise + (t % 2), cruise - (t % 2), 1);
        db_pose_estimator_predict(&ref, cruise + (t % 2), cruise - (t % 2), 1);
    }
    CHECK(est.P[2][2] - ref.P[2][2] - h0 < 0.5f * DEG * DEG, "1 s at 700 mm/s with count jitter adds under 0.5 deg^2 of slip noise, %.2f", (est.P[2][2] - ref.P[2][2] - h0) / (DEG * DEG));
    // Reset the heading variance so the stop's position share is not scaled by the cruise's
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            est.P[i][j] = ref.P[i][j] = (i == j) ? ((i == 2) ? DEG * DEG : 4.0f) : 0;
        }
    }
    for (int t = 0; t < 10; t++) {
        int32_t c = cruise * (9 - t) / 10;
        db_pose_estimator_predict(&est, c, c, 1);
        db_pose_estimator_predict(&ref, c, c, 1);
    }
    for (int t = 0; t < 20; t++) {
        db_pose_estimator_predict(&est, 0, 0, 1);
        db_pose_estimator_predict(&ref, 0, 0, 1);
    }
    CHECK(est.P[0][0] - ref.P[0][0] > 40.0f, "a hard stop from 700 mm/s adds over 40 mm^2 per axis, %.1f", est.P[0][0] - ref.P[0][0]);
    CHECK((est.P[2][2] - ref.P[2][2]) / (DEG * DEG) > 9.0f, "and over (3 deg)^2 of heading variance, %.1f deg^2", (est.P[2][2] - ref.P[2][2]) / (DEG * DEG));
}

/// A sprint from rest and a hard stop in which the odometry counts slip_mm more
/// travel than the ground saw, along the heading and across it; returns the
/// estimator 0.5 s after the stop
static void _sprint_and_slip(db_pose_estimator_t *est, const db_pose_estimator_conf_t *conf, float speed, float slip_mm, float lateral_mm, robot_t *r) {
    db_pose_estimator_init(est, conf);
    db_pose_estimator_seed(est, r->x, r->y, r->theta / DEG, 1);
    _run(est, r, 0, 0, 100, 1, LH2_NOISE_SD_MM);
    int32_t  cruise = (int32_t)lroundf(speed * 0.01f / DB_MM_PER_COUNT);
    uint32_t tick   = 0;
    // 0.3 s at speed, then a 10-tick stop over which the odometry over-counts
    for (int t = 0; t < 40; t++) {
        int32_t c     = (t < 30) ? cruise : cruise * (39 - t) / 10;
        int32_t extra = (t < 30) ? 0 : (int32_t)lroundf(slip_mm / DB_MM_PER_COUNT / 10.0f) * (speed < 0 ? -1 : 1);
        _robot_step(r, c, c);
        if (t >= 30) {
            r->x += lateral_mm / 10.0f * cosf(r->theta);
            r->y += lateral_mm / 10.0f * sinf(r->theta);
        }
        db_pose_estimator_predict(est, c + extra, c + extra, 1);
        if (++tick % TICKS_PER_FIX == 0) {
            float zx, zy;
            _robot_sensor(r, LH2_NOISE_SD_MM, &zx, &zy);
            db_pose_estimator_update(est, zx, zy);
        }
    }
    for (int t = 0; t < 50; t++) {
        _robot_step(r, 0, 0);
        db_pose_estimator_predict(est, 0, 0, 1);
        if (++tick % TICKS_PER_FIX == 0) {
            float zx, zy;
            _robot_sensor(r, LH2_NOISE_SD_MM, &zx, &zy);
            db_pose_estimator_update(est, zx, zy);
        }
    }
}

static void test_hard_stop_slip_is_not_a_kidnap(void) {
    // As on the floor: after a 500 to 600 mm/s sprint and a hard stop, the
    // estimate sat 25 to 45 mm off steady fixes with the wheels still
    const float speeds[]  = { 600, -600, 500, -500 };
    const float slips[]   = { 35, 40, 30, 45 };
    const float lateral[] = { 0, 0, 25, -20 };
    for (unsigned i = 0; i < 4; i++) {
        db_pose_estimator_t est;
        robot_t             r = { .x = 1000, .y = 1000, .theta = 90 * DEG, .seed = 30 + i, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
        _sprint_and_slip(&est, &_conf, speeds[i], slips[i], lateral[i], &r);
        float sx = 0, sy = 0, px, py;
        _robot_sensor(&r, 0, &px, &py);
        bool tracking = db_pose_estimator_sensor(&est, &sx, &sy);
        CHECK(tracking && est.kidnaps == 0 && est.seeds == 0, "%+.0f mm/s, %.0f mm slip: still tracking, no kidnap, status %d kidnaps %u", speeds[i], slips[i], est.status, est.kidnaps);
        if (lateral[i] == 0) {
            CHECK(est.reanchors == 0, "%+.0f mm/s, %.0f mm slip: the slip noise lets the fixes in, no re-anchor needed, %u", speeds[i], slips[i], est.reanchors);
        }
        CHECK(hypotf(sx - px, sy - py) < 5.0f, "%+.0f mm/s, %.0f mm slip: on the fixes 0.5 s after the stop, %.1f mm off", speeds[i], slips[i], hypotf(sx - px, sy - py));
        // A sideways slide is partly read as a turn until the robot moves again
        float heading_tol = (lateral[i] == 0) ? 4.0f : 8.0f;
        CHECK(_angle_error_deg(est.theta, r.theta) < heading_tol, "%+.0f mm/s, %.0f mm slip: heading kept, %.1f deg off", speeds[i], slips[i], _angle_error_deg(est.theta, r.theta));
    }
}

static void test_reanchor_after_driving(void) {
    // Without the slip noise the gate rejects the fixes; the robot drove a
    // moment ago, so it re-anchors, keeping the heading, instead of a kidnap
    db_pose_estimator_conf_t conf     = _conf;
    conf.q_pos_slip_mm2_per_mm_s      = 0;
    conf.q_heading_slip_deg2_per_mm_s = 0;
    const float speeds[]              = { 600, -600 };
    for (unsigned i = 0; i < 2; i++) {
        db_pose_estimator_t est;
        robot_t             r = { .x = 1000, .y = 1000, .theta = 90 * DEG, .seed = 40 + i, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
        _sprint_and_slip(&est, &conf, speeds[i], 40, 0, &r);
        float sx = 0, sy = 0, px, py;
        _robot_sensor(&r, 0, &px, &py);
        bool tracking = db_pose_estimator_sensor(&est, &sx, &sy);
        CHECK(tracking && est.kidnaps == 0 && est.reanchors == 1, "%+.0f mm/s, no slip noise: re-anchored, not kidnapped, status %d kidnaps %u reanchors %u", speeds[i], est.status, est.kidnaps, est.reanchors);
        CHECK(hypotf(sx - px, sy - py) < 5.0f, "%+.0f mm/s, no slip noise: on the fixes, %.1f mm off", speeds[i], hypotf(sx - px, sy - py));
        CHECK(_angle_error_deg(est.theta, r.theta) < 4.0f, "%+.0f mm/s, no slip noise: heading kept, %.1f deg off", speeds[i], _angle_error_deg(est.theta, r.theta));
    }
    // With neither, as before the fix, the same stop is taken for a kidnap
    conf.kidnap_settle_ticks = 0;
    conf.reanchor_mm         = 0;
    db_pose_estimator_t est;
    robot_t             r = { .x = 1000, .y = 1000, .theta = 90 * DEG, .seed = 42, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    _sprint_and_slip(&est, &conf, -600, 40, 0, &r);
    CHECK(est.kidnaps == 1, "without the guard the stop reads as a kidnap, kidnaps %u", est.kidnaps);
}

static void test_moved_right_after_driving(void) {
    // Carried 150 mm the moment it stopped: too far to be slip, so still a kidnap
    db_pose_estimator_t est;
    robot_t             r   = { .x = 1000, .y = 1000, .theta = 0, .seed = 44, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    int32_t             run = (int32_t)lroundf(300.0f * 0.01f / DB_MM_PER_COUNT);
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 0, 1);
    _run(&est, &r, run, run, 50, 1, LH2_NOISE_SD_MM);
    _run(&est, &r, 0, 0, 5, 1, LH2_NOISE_SD_MM);
    r.x += 150;
    _run(&est, &r, 0, 0, 45, 1, LH2_NOISE_SD_MM);
    CHECK(est.kidnaps == 1 && est.reanchors == 0, "a 150 mm move right after driving is a kidnap, kidnaps %u reanchors %u", est.kidnaps, est.reanchors);
}

static void test_turn_noise_grows_with_turn_speed(void) {
    // The same wheel travel difference, turned slowly or fast
    int32_t             counts = 1000;
    float               dd_mm  = 2.0f * counts * DB_MM_PER_COUNT;
    uint32_t            slow   = (uint32_t)ceilf(dd_mm / (0.5f * _conf.turn_speed_ref_mm_s) * 100.0f);
    uint32_t            slower = 2 * slow;
    db_pose_estimator_t a, b, c;
    db_pose_estimator_init(&a, &_conf);
    db_pose_estimator_init(&b, &_conf);
    db_pose_estimator_init(&c, &_conf);
    db_pose_estimator_seed(&a, 0, 0, 0, 0);
    db_pose_estimator_seed(&b, 0, 0, 0, 0);
    db_pose_estimator_seed(&c, 0, 0, 0, 0);
    db_pose_estimator_predict(&a, counts, -counts, slower);
    db_pose_estimator_predict(&b, counts, -counts, slow);
    db_pose_estimator_predict(&c, counts, -counts, slow / 4);
    CHECK(fabsf(a.P[2][2] - b.P[2][2]) < 1e-3f * a.P[2][2], "below the reference turn speed the turn speed does not matter: %.6g against %.6g", a.P[2][2], b.P[2][2]);
    CHECK(c.P[2][2] > 1.5f * b.P[2][2], "above it, faster turning adds more heading noise: %.6g against %.6g", c.P[2][2], b.P[2][2]);
}

static void test_seed_carried_to_present(void) {
    // A chain seeds from fixes DB_POSE_ESTIMATOR_FIX_AGE_TICKS old; the pose it
    // sets is the present one, not the one at capture
    int32_t fast = (int32_t)lroundf(300.0f * 0.01f / DB_MM_PER_COUNT);
    int32_t spin = (int32_t)lroundf(200.0f * 0.01f / DB_MM_PER_COUNT);
    for (int spinning = 0; spinning <= 1; spinning++) {
        db_pose_estimator_t est;
        robot_t             r  = { .x = 1000, .y = 1000, .theta = 30 * DEG, .seed = 11, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
        int32_t             cl = spinning ? spin : fast;
        int32_t             cr = spinning ? -spin : fast;
        db_pose_estimator_init(&est, &_conf);
        for (uint32_t t = 1; t <= 50 * TICKS_PER_FIX && est.seeds == 0; t++) {
            _robot_step(&r, cl, cr);
            db_pose_estimator_predict(&est, cl, cr, 1);
            if ((t % TICKS_PER_FIX) == 0) {
                float zx, zy;
                _robot_sensor(&r, 0, &zx, &zy);
                db_pose_estimator_update(&est, zx, zy);
            }
        }
        CHECK(est.seeds == 1, "%s seeds, seeds %u", spinning ? "a spin" : "a straight", est.seeds);
        CHECK(hypotf(est.x - r.x, est.y - r.y) < 0.5f, "%s seeds the present axle, %.2f mm off", spinning ? "a spin" : "a straight", hypotf(est.x - r.x, est.y - r.y));
        CHECK(_angle_error_deg(est.theta, r.theta) < 0.5f, "%s seeds the present heading, %.2f deg off", spinning ? "a spin" : "a straight", _angle_error_deg(est.theta, r.theta));
    }
}

static void test_chain_survives_outlier(void) {
    // One wild fix while acquiring restarts the chain; the heading then
    // acquired carries nothing of it
    db_pose_estimator_t est;
    robot_t             r = { .x = 1500, .y = 900, .theta = -120 * DEG, .seed = 13, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    _run(&est, &r, 10, 10, 2 * TICKS_PER_FIX, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_SEEDING && est.chain_count == 2, "chaining, status %d count %u", est.status, est.chain_count);
    float zx, zy;
    _robot_sensor(&r, 0, &zx, &zy);
    CHECK(db_pose_estimator_update(&est, zx + 150, zy - 100) == DB_POSE_ESTIMATOR_REJECTED, "a wild fix breaks the chain");
    _run(&est, &r, 10, 10, 150, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING && est.seeds == 1, "seeded after the wild fix, status %d seeds %u", est.status, est.seeds);
    CHECK(_angle_error_deg(est.theta, r.theta) < 5.0f, "heading within 5 deg, %.2f off", _angle_error_deg(est.theta, r.theta));
}

static void test_long_carry_times_out(void) {
    // Carried for longer than the timeout with the wheels still: every fix
    // moves more than the seed tolerance, so no kidnap; the pose goes LOST,
    // the heading stays unknown at rest, and motion reseeds it
    db_pose_estimator_t est;
    robot_t             r = { .x = 1000, .y = 1000, .theta = 0, .seed = 14, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 0, 2);
    _run(&est, &r, 0, 0, 50, 1, LH2_NOISE_SD_MM);
    for (int i = 0; i < 15; i++) {
        r.x += 40;
        r.theta += 6 * DEG;
        _run(&est, &r, 0, 0, TICKS_PER_FIX, 1, LH2_NOISE_SD_MM);
    }
    CHECK(est.status == DB_POSE_ESTIMATOR_LOST && est.kidnaps == 0, "a carry longer than the timeout is lost, not kidnapped, status %d kidnaps %u", est.status, est.kidnaps);
    _run(&est, &r, 0, 0, 100, 1, LH2_NOISE_SD_MM);
    float h = 0;
    CHECK(!db_pose_estimator_heading_deg(&est, &h) && est.seeds == 0, "no heading at rest after the carry, status %d", est.status);
    _run(&est, &r, 10, 10, 150, 1, LH2_NOISE_SD_MM);
    CHECK(est.status == DB_POSE_ESTIMATOR_TRACKING && est.seeds == 1, "motion reseeds, status %d seeds %u", est.status, est.seeds);
    CHECK(_angle_error_deg(est.theta, r.theta) < 5.0f, "heading within 5 deg, %.2f off", _angle_error_deg(est.theta, r.theta));
}

static void test_covariance_stays_positive_definite(void) {
    // An hour at rest at 10 Hz: fixes shrink P toward rank 1, which the
    // non-Joseph update let go indefinite in float32
    db_pose_estimator_t est;
    robot_t             r = { .x = 1000, .y = 1000, .theta = 0, .seed = 12, .fix_age = DB_POSE_ESTIMATOR_FIX_AGE_TICKS };
    db_pose_estimator_init(&est, &_conf);
    db_pose_estimator_seed(&est, r.x, r.y, 0, 5);
    uint32_t indefinite = 0;
    for (uint32_t t = 1; t <= 3600U * 100U; t++) {
        db_pose_estimator_predict(&est, 0, 0, 1);
        if ((t % TICKS_PER_FIX) == 0) {
            float zx, zy;
            _robot_sensor(&r, LH2_NOISE_SD_MM, &zx, &zy);
            db_pose_estimator_update(&est, zx, zy);
            const float(*P)[3] = est.P;
            double m2          = (double)P[0][0] * P[1][1] - (double)P[0][1] * P[1][0];
            double m3          = P[0][0] * ((double)P[1][1] * P[2][2] - (double)P[1][2] * P[2][1]) - P[0][1] * ((double)P[1][0] * P[2][2] - (double)P[1][2] * P[2][0]) + P[0][2] * ((double)P[1][0] * P[2][1] - (double)P[1][1] * P[2][0]);
            if (!(P[0][0] > 0 && m2 > 0 && m3 > 0) || P[0][1] != P[1][0] || P[0][2] != P[2][0] || P[1][2] != P[2][1]) {
                indefinite++;
            }
        }
    }
    CHECK(indefinite == 0, "P stays symmetric positive definite over an hour at rest, %u updates not", indefinite);
    CHECK(est.rejected == 0, "no fix is rejected at rest, got %u", est.rejected);
}

int main(void) {
    test_straight_prediction();
    test_arc_prediction();
    test_spin_prediction();
    test_effective_track();
    test_lever_arm_sensor();
    test_lever_arm_spin_update();
    test_converges_from_wrong_heading();
    test_acquire_heading_from_motion();
    test_acquire_heading_from_spin();
    test_gate_rejects_outlier();
    test_kidnap_reseeds();
    test_outlier_still_no_kidnap();
    test_rejected_while_driving_times_out();
    test_occlusion_keeps_heading();
    test_fix_age_compensated();
    test_noise_scales_with_distance();
    test_slip_noise_on_hard_stop();
    test_hard_stop_slip_is_not_a_kidnap();
    test_reanchor_after_driving();
    test_moved_right_after_driving();
    test_turn_noise_grows_with_turn_speed();
    test_seed_carried_to_present();
    test_chain_survives_outlier();
    test_long_carry_times_out();
    test_covariance_stays_positive_definite();
    printf("%d passed, %d failed\n", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
