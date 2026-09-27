/**
 * @file
 * @brief  Host test of drv/dotbot_control, closed through a duty-driven plant
 *
 * The plant inverts the wheel loop's running line with a 50 ms lag and a
 * breakaway, brakes to a stop with the same lag, emits whole encoder counts,
 * and is seen by LH2 at 10 Hz with 2 ticks of age and 1 mm of noise.
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dotbot_control.h"
#include "geometry.h"

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

//=========================== plant ============================================

#define PLANT_TAU_S     (0.05f)
#define PLANT_DT_S      (0.01f)
#define PLANT_U_RUN     (32.0f)
#define PLANT_K_RUN     (0.097f)
#define PLANT_U_BREAK   (40.0f)
#define PLANT_FIX_AGE   (2U)
#define PLANT_FIX_EVERY (10U)

typedef struct {
    float    x;            ///< truth, axle midpoint, mm
    float    y;            ///< truth, axle midpoint, mm
    float    heading_deg;  ///< truth, 0 facing +y, clockwise positive
    float    v_left;       ///< mm/s
    float    v_right;      ///< mm/s
    float    frac_left;    ///< counts not yet emitted
    float    frac_right;   ///< counts not yet emitted
    int8_t   pwm_left;
    int8_t   pwm_right;
    bool     brake_left;
    bool     brake_right;
    float    noise_mm;
    uint32_t rng;
    uint32_t tick;
    float    px[PLANT_FIX_AGE + 1];  ///< photodiode history, newest first
    float    py[PLANT_FIX_AGE + 1];
    uint32_t fix_sequence;
    uint32_t fix_x;
    uint32_t fix_y;
} plant_t;

static float _uniform(plant_t *p) {
    p->rng = p->rng * 1664525u + 1013904223u;
    return ((float)(p->rng >> 8) + 0.5f) / 16777216.0f;
}

static float _gauss(plant_t *p) {
    float u1 = _uniform(p);
    float u2 = _uniform(p);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

static void _forward(float heading_deg, float *fx, float *fy) {
    float h = heading_deg * (float)M_PI / 180.0f;
    *fx     = -sinf(h);
    *fy     = cosf(h);
}

static void _plant_init(plant_t *p, float x, float y, float heading_deg, float noise_mm, uint32_t seed) {
    memset(p, 0, sizeof(*p));
    p->x           = x;
    p->y           = y;
    p->heading_deg = heading_deg;
    p->noise_mm    = noise_mm;
    p->rng         = seed ? seed : 1;
    float fx;
    float fy;
    _forward(heading_deg, &fx, &fy);
    for (unsigned i = 0; i <= PLANT_FIX_AGE; i++) {
        p->px[i] = x + DB_LH2_LEVER_ARM_EFFECTIVE * fx;
        p->py[i] = y + DB_LH2_LEVER_ARM_EFFECTIVE * fy;
    }
}

static float _plant_target(float pwm, float v) {
    float a = fabsf(pwm);
    if (a <= PLANT_U_RUN || (fabsf(v) < 1.0f && a < PLANT_U_BREAK)) {
        return 0.0f;
    }
    return copysignf((a - PLANT_U_RUN) / PLANT_K_RUN, pwm);
}

static void _plant_step(plant_t *p, db_control_input_t *in) {
    float gain = 1.0f - expf(-PLANT_DT_S / PLANT_TAU_S);
    float tl   = p->brake_left ? 0.0f : _plant_target(p->pwm_left, p->v_left);
    float tr   = p->brake_right ? 0.0f : _plant_target(p->pwm_right, p->v_right);
    float vl   = p->v_left + gain * (tl - p->v_left);
    float vr   = p->v_right + gain * (tr - p->v_right);
    float dl   = 0.5f * (p->v_left + vl) * PLANT_DT_S;
    float dr   = 0.5f * (p->v_right + vr) * PLANT_DT_S;
    p->v_left  = vl;
    p->v_right = vr;

    float d      = 0.5f * (dl + dr);
    float dtheta = (dl - dr) / db_track_effective_mm(dl, dr) * 180.0f / (float)M_PI;
    float fx;
    float fy;
    _forward(p->heading_deg + dtheta / 2.0f, &fx, &fy);
    p->x += d * fx;
    p->y += d * fy;
    p->heading_deg += dtheta;
    if (p->heading_deg >= 180.0f) {
        p->heading_deg -= 360.0f;
    }
    if (p->heading_deg < -180.0f) {
        p->heading_deg += 360.0f;
    }

    p->frac_left += dl / DB_MM_PER_COUNT;
    p->frac_right += dr / DB_MM_PER_COUNT;
    in->counts_left  = (int32_t)truncf(p->frac_left);
    in->counts_right = (int32_t)truncf(p->frac_right);
    p->frac_left -= (float)in->counts_left;
    p->frac_right -= (float)in->counts_right;

    memmove(&p->px[1], &p->px[0], PLANT_FIX_AGE * sizeof(float));
    memmove(&p->py[1], &p->py[0], PLANT_FIX_AGE * sizeof(float));
    _forward(p->heading_deg, &fx, &fy);
    p->px[0] = p->x + DB_LH2_LEVER_ARM_EFFECTIVE * fx;
    p->py[0] = p->y + DB_LH2_LEVER_ARM_EFFECTIVE * fy;

    p->tick++;
    if (p->tick % PLANT_FIX_EVERY == 0) {
        float nx = p->noise_mm > 0 ? p->noise_mm * _gauss(p) : 0;
        float ny = p->noise_mm > 0 ? p->noise_mm * _gauss(p) : 0;
        p->fix_sequence++;
        p->fix_x = (uint32_t)lroundf(fmaxf(0, p->px[PLANT_FIX_AGE] + nx));
        p->fix_y = (uint32_t)lroundf(fmaxf(0, p->py[PLANT_FIX_AGE] + ny));
    }
    in->fix_sequence  = p->fix_sequence;
    in->fix_x         = p->fix_x;
    in->fix_y         = p->fix_y;
    in->elapsed_ticks = 1;
}

static void _plant_apply(plant_t *p, const db_control_output_t *out) {
    if (out->write) {
        p->pwm_left    = out->pwm_left;
        p->pwm_right   = out->pwm_right;
        p->brake_left  = out->brake_left;
        p->brake_right = out->brake_right;
    }
}

//=========================== helpers ==========================================

typedef struct {
    db_control_t        control;
    plant_t             plant;
    db_control_output_t out;
    uint8_t             states[32];  ///< steering states entered, in order
    uint32_t            state_count;
    uint32_t            arrived_tick;  ///< 0 until the steering first reports ARRIVED
    uint32_t            heading_tick;  ///< 0 until the steering first leaves NO_HEADING
} sim_t;

static void _sim_init(sim_t *s, int seeded) {
    memset(s, 0, sizeof(*s));
    _plant_init(&s->plant, 1000.0f, 1000.0f, 0.0f, 1.0f, 12345u);
    db_control_init(&s->control, &db_control_default_conf);
    if (seeded) {
        db_pose_estimator_seed(&s->control.estimator, 1000.0f, 1000.0f, 0.0f, 2.0f);
    }
}

static void _sim_run(sim_t *s, uint32_t ticks) {
    for (uint32_t i = 0; i < ticks; i++) {
        db_control_input_t in;
        _plant_step(&s->plant, &in);
        db_control_tick(&s->control, &in, &s->out);
        _plant_apply(&s->plant, &s->out);
        uint8_t state = (uint8_t)s->control.steering.state;
        if (state != DB_STEERING_IDLE && (s->state_count == 0 || s->states[s->state_count - 1] != state) && s->state_count < sizeof(s->states)) {
            s->states[s->state_count++] = state;
        }
        if (state != DB_STEERING_IDLE && state != DB_STEERING_NO_HEADING && s->heading_tick == 0) {
            s->heading_tick = s->control.tick;
        }
        if (state == DB_STEERING_ARRIVED && s->arrived_tick == 0) {
            s->arrived_tick = s->control.tick;
        }
    }
}

static size_t _waypoints(uint8_t *buf, const uint32_t (*points)[2], uint8_t count, uint16_t threshold, uint8_t batch_id) {
    size_t length = 0;
    buf[length++] = DB_PROTOCOL_LH2_WAYPOINTS;
    memcpy(&buf[length], &threshold, sizeof(threshold));
    length += sizeof(threshold);
    buf[length++] = count;
    for (uint8_t i = 0; i < count; i++) {
        protocol_lh2_location_t point = { .x = points[i][0], .y = points[i][1] };
        memcpy(&buf[length], &point, sizeof(point));
        length += sizeof(point);
    }
    protocol_lh2_waypoints_trailer_t trailer = { .batch_id = batch_id };
    memcpy(&buf[length], &trailer, sizeof(trailer));
    length += sizeof(trailer);
    for (uint8_t i = 0; i < count; i++) {
        int16_t heading = DB_WAYPOINT_NO_HEADING;
        memcpy(&buf[length], &heading, sizeof(heading));
        length += sizeof(heading);
    }
    return length;
}

static const uint32_t _points[3][2] = { { 1000, 1500 }, { 1500, 1500 }, { 1500, 1000 } };

static void _send_batch(sim_t *s, uint8_t batch_id) {
    uint8_t buf[DB_CONTROL_RX_MAX_BYTES];
    size_t  length = _waypoints(buf, _points, 3, 30, batch_id);
    db_control_rx(&s->control, buf, length);
}

static void _send_velocity(sim_t *s, int16_t left, int16_t right) {
    uint8_t                           buf[1 + sizeof(protocol_wheel_velocity_command_t)];
    protocol_wheel_velocity_command_t command = { .left_mm_s = left, .right_mm_s = right };
    buf[0]                                    = DB_PROTOCOL_CMD_WHEEL_VELOCITY;
    memcpy(&buf[1], &command, sizeof(command));
    db_control_rx(&s->control, buf, sizeof(buf));
}

static void _send_max_speed(sim_t *s, uint16_t mm_s) {
    uint8_t                      buf[1 + sizeof(protocol_max_speed_command_t)];
    protocol_max_speed_command_t command = { .max_speed_mm_s = mm_s };
    buf[0]                               = DB_PROTOCOL_CMD_MAX_SPEED;
    memcpy(&buf[1], &command, sizeof(command));
    db_control_rx(&s->control, buf, sizeof(buf));
}

static float _miss(const sim_t *s, uint32_t x, uint32_t y) {
    return hypotf(s->plant.x - (float)x, s->plant.y - (float)y);
}

static int _states_equal(const sim_t *s, const uint8_t *expected, uint32_t count) {
    return s->state_count == count && memcmp(s->states, expected, count) == 0;
}

static void _print_states(const sim_t *s) {
    printf("  states:");
    for (uint32_t i = 0; i < s->state_count; i++) {
        printf(" %u", s->states[i]);
    }
    printf("\n");
}

//=========================== tests ============================================

static void test_batch_seeded(void) {
    sim_t s;
    _sim_init(&s, 1);
    _send_batch(&s, 1);
    _sim_run(&s, 1500);

    static const uint8_t expected[] = { DB_STEERING_NO_HEADING, DB_STEERING_DRIVE, DB_STEERING_ALIGN, DB_STEERING_DRIVE, DB_STEERING_ALIGN, DB_STEERING_DRIVE, DB_STEERING_ARRIVED };
    CHECK(_states_equal(&s, expected, sizeof(expected)), "seeded: drives, turns, drives, turns, drives, arrives");
    CHECK(s.heading_tick == 10, "seeded: leaves NO_HEADING at the first steering step, tick %u", s.heading_tick);
    if (!_states_equal(&s, expected, sizeof(expected))) {
        _print_states(&s);
    }
    CHECK(s.arrived_tick > 0 && s.arrived_tick < 900, "seeded: arrives within 9 s, took %u ticks", s.arrived_tick);
    CHECK(_miss(&s, 1500, 1000) < 30.0f + 5.0f, "seeded: stops within the threshold, missed by %.1f", _miss(&s, 1500, 1000));

    db_control_report_t report;
    db_control_report(&s.control, &report);
    CHECK(report.status == DB_WAYPOINTS_ARRIVED && report.reason == 0, "seeded: reports ARRIVED, status %u", report.status);
    CHECK(report.waypoint_index == 3 && report.waypoint_count == 3, "seeded: index is the count once arrived, %u/%u", report.waypoint_index, report.waypoint_count);
    CHECK(report.batch_id == 1, "seeded: reports the batch id, %u", report.batch_id);
    CHECK(report.control_mode == ControlManual, "seeded: manual once arrived");
    CHECK(report.estimator_status == DB_POSE_ESTIMATOR_TRACKING, "seeded: still tracking");
    CHECK(fabsf(report.axle_x_mm - s.plant.x) < 5.0f && fabsf(report.axle_y_mm - s.plant.y) < 5.0f, "seeded: estimate within 5 mm of truth, %.1f %.1f vs %.1f %.1f", report.axle_x_mm, report.axle_y_mm, s.plant.x, s.plant.y);
    CHECK(report.axle_x == (uint16_t)lroundf(report.axle_x_mm), "seeded: advertised axle is the estimate");
    CHECK(report.brake_left && report.brake_right, "seeded: braked once arrived");
}

static void test_batch_from_boot(void) {
    sim_t s;
    _sim_init(&s, 0);
    _send_batch(&s, 1);
    _sim_run(&s, 1500);

    static const uint8_t expected[] = { DB_STEERING_NO_HEADING, DB_STEERING_ALIGN, DB_STEERING_DRIVE, DB_STEERING_ALIGN, DB_STEERING_DRIVE, DB_STEERING_ALIGN, DB_STEERING_DRIVE, DB_STEERING_ARRIVED };
    CHECK(_states_equal(&s, expected, sizeof(expected)), "boot: spins for a heading, then turns to the first leg and drives the batch");
    if (!_states_equal(&s, expected, sizeof(expected))) {
        _print_states(&s);
    }
    CHECK(s.heading_tick > 10, "boot: spins past the first steering step, until tick %u", s.heading_tick);
    CHECK(s.states[s.state_count - 1] == DB_STEERING_ARRIVED, "boot: arrives");
    CHECK(s.arrived_tick > 0 && s.arrived_tick < 1000, "boot: arrives within 10 s, took %u ticks", s.arrived_tick);
    CHECK(_miss(&s, 1500, 1000) < 30.0f + 5.0f, "boot: stops within the threshold, missed by %.1f", _miss(&s, 1500, 1000));
}

static void test_batch_dedup(void) {
    sim_t s;
    _sim_init(&s, 1);
    _send_batch(&s, 7);
    _sim_run(&s, 1500);
    CHECK(s.control.steering.state == DB_STEERING_ARRIVED, "dedup: arrived first");

    _send_batch(&s, 7);
    _sim_run(&s, 1);
    CHECK(s.control.steering.state == DB_STEERING_ARRIVED, "dedup: a resent batch id is ignored, state %d", s.control.steering.state);

    _send_batch(&s, 8);
    _sim_run(&s, 1);
    CHECK(s.control.steering.state != DB_STEERING_ARRIVED && s.control.steering.index == 0, "dedup: a new batch id restarts, state %d", s.control.steering.state);
    CHECK(s.control.batch_id == 8, "dedup: the new id is kept");

    sim_t z;
    _sim_init(&z, 1);
    _send_batch(&z, 0);
    _sim_run(&z, 1500);
    _send_batch(&z, 0);
    _sim_run(&z, 1);
    CHECK(z.control.steering.state != DB_STEERING_ARRIVED, "dedup: batch id 0 is never deduplicated, state %d", z.control.steering.state);
}

static void test_max_speed(void) {
    sim_t               s;
    db_control_report_t report;
    _sim_init(&s, 1);

    db_control_report(&s.control, &report);
    CHECK(report.max_speed_mm_s == DB_STEERING_V_MAX_MM_S && report.max_speed_10mm == 30, "max speed: the default at boot, %.0f", report.max_speed_mm_s);
    _send_max_speed(&s, 5);
    db_control_report(&s.control, &report);
    CHECK(report.max_speed_mm_s == 20.0f, "max speed: raised to the floor, %.0f", report.max_speed_mm_s);
    _send_max_speed(&s, 1000);
    db_control_report(&s.control, &report);
    CHECK(report.max_speed_mm_s == 700.0f && report.max_speed_10mm == 70, "max speed: cut to the ceiling, %.0f", report.max_speed_mm_s);
    _send_max_speed(&s, 150);
    db_control_report(&s.control, &report);
    CHECK(report.max_speed_mm_s == 150.0f && report.max_speed_10mm == 15, "max speed: in range kept, %.0f", report.max_speed_mm_s);
    _send_max_speed(&s, 0);
    db_control_report(&s.control, &report);
    CHECK(report.max_speed_mm_s == DB_STEERING_V_MAX_MM_S, "max speed: 0 restores the default, %.0f", report.max_speed_mm_s);

    uint8_t short_packet[2] = { DB_PROTOCOL_CMD_MAX_SPEED, 50 };
    db_control_rx(&s.control, short_packet, sizeof(short_packet));
    db_control_report(&s.control, &report);
    CHECK(report.max_speed_mm_s == DB_STEERING_V_MAX_MM_S, "max speed: a short command is ignored");
}

static void test_abort_reasons(void) {
    sim_t               s;
    db_control_report_t report;

    _sim_init(&s, 1);
    _send_batch(&s, 1);
    _sim_run(&s, 50);
    _send_velocity(&s, 100, 100);
    _sim_run(&s, 1);
    db_control_report(&s.control, &report);
    CHECK(report.status == DB_WAYPOINTS_ABORTED && report.reason == DB_WAYPOINTS_ABORT_DIRECT, "abort: wheel velocity aborts DIRECT, %u/%u", report.status, report.reason);
    CHECK(report.drive_mode == DB_CONTROL_DRIVE_VELOCITY, "abort: velocity owns the motors");

    _sim_init(&s, 1);
    _send_batch(&s, 1);
    _sim_run(&s, 50);
    uint8_t control_mode[2] = { DB_PROTOCOL_CONTROL_MODE, ControlManual };
    db_control_rx(&s.control, control_mode, sizeof(control_mode));
    _sim_run(&s, 1);
    db_control_report(&s.control, &report);
    CHECK(report.status == DB_WAYPOINTS_ABORTED && report.reason == DB_WAYPOINTS_ABORT_CONTROL_MODE, "abort: control mode aborts CONTROL_MODE, %u/%u", report.status, report.reason);
    CHECK(report.drive_mode == DB_CONTROL_DRIVE_IDLE, "abort: control mode idles");

    _sim_init(&s, 1);
    _send_batch(&s, 1);
    _sim_run(&s, 50);
    uint8_t buf[DB_CONTROL_RX_MAX_BYTES];
    size_t  length = _waypoints(buf, _points, 0, 30, 2);
    db_control_rx(&s.control, buf, length);
    _sim_run(&s, 1);
    db_control_report(&s.control, &report);
    CHECK(report.status == DB_WAYPOINTS_ABORTED && report.reason == DB_WAYPOINTS_ABORT_STOP, "abort: an empty batch aborts STOP, %u/%u", report.status, report.reason);
    CHECK(report.batch_id == 2, "abort: an empty batch's id is kept, %u", report.batch_id);
    CHECK(report.drive_mode == DB_CONTROL_DRIVE_IDLE, "abort: an empty batch idles");

    // A later command leaves the reason of the batch it did not stop
    _send_velocity(&s, 100, 100);
    _sim_run(&s, 1);
    db_control_report(&s.control, &report);
    CHECK(report.reason == DB_WAYPOINTS_ABORT_STOP, "abort: a later command keeps the reason, %u", report.reason);
}

static void test_raw_and_deadman(void) {
    sim_t s;
    _sim_init(&s, 1);

    uint8_t                     raw[1 + sizeof(protocol_move_raw_command_t)];
    protocol_move_raw_command_t command = { .left_y = 127, .right_y = -64 };
    raw[0]                              = DB_PROTOCOL_CMD_MOVE_RAW;
    memcpy(&raw[1], &command, sizeof(command));
    db_control_rx(&s.control, raw, sizeof(raw));
    _sim_run(&s, 1);
    CHECK(s.out.write && s.out.pwm_left == 100 && s.out.pwm_right == -50 && !s.out.brake_left, "raw: duty written on the next tick, %d %d", s.out.pwm_left, s.out.pwm_right);
    _sim_run(&s, 1);
    CHECK(!s.out.write, "raw: nothing written between commands");

    // Commanded before tick 1; checks fall every 20 ticks, and the first one past 52 is tick 60
    uint32_t stopped = 0;
    for (uint32_t i = 0; i < 100 && stopped == 0; i++) {
        _sim_run(&s, 1);
        if (s.control.drive_mode == DB_CONTROL_DRIVE_IDLE) {
            stopped = s.control.tick;
        }
    }
    CHECK(stopped == 60, "deadman: raw stops at tick 60, stopped at %u", stopped);
    CHECK(s.out.write && s.out.brake_left && s.out.brake_right, "deadman: the stop brakes");

    _sim_init(&s, 1);
    _sim_run(&s, 5);
    _send_velocity(&s, 100, 100);
    for (uint32_t i = 0; i < 50; i++) {
        _sim_run(&s, 1);
        if (i % 20 == 0) {
            _send_velocity(&s, 100, 100);
        }
    }
    CHECK(s.control.drive_mode == DB_CONTROL_DRIVE_VELOCITY, "deadman: resent commands keep driving");
    _sim_run(&s, 100);
    CHECK(s.control.drive_mode == DB_CONTROL_DRIVE_IDLE, "deadman: velocity stops once they cease");

    _sim_init(&s, 1);
    _send_batch(&s, 1);
    _sim_run(&s, 200);
    CHECK(s.control.drive_mode == DB_CONTROL_DRIVE_WAYPOINT, "deadman: waypoints need no resending");
}

static void test_velocity_clamped(void) {
    sim_t s;
    _sim_init(&s, 1);
    _send_velocity(&s, 2000, -2000);
    CHECK(s.control.wheel_left.setpoint == 700.0f && s.control.wheel_right.setpoint == -700.0f, "velocity: clamped to 700 mm/s, %.0f %.0f", s.control.wheel_left.setpoint, s.control.wheel_right.setpoint);
}

static void test_oversized_ignored(void) {
    sim_t   s;
    uint8_t buf[DB_CONTROL_RX_MAX_BYTES + 1];
    _sim_init(&s, 1);
    size_t length = _waypoints(buf, _points, 3, 30, 1);
    memset(&buf[length], 0, sizeof(buf) - length);
    db_control_rx(&s.control, buf, sizeof(buf));
    CHECK(s.control.drive_mode == DB_CONTROL_DRIVE_IDLE && s.control.batch_id == 0, "rx: a command longer than the mailbox is ignored");
}

static void test_advertisement(void) {
    sim_t s;
    _sim_init(&s, 1);
    _send_batch(&s, 3);

    uint32_t adverts = 0;
    uint8_t  buf[DB_CONTROL_ADVERTISEMENT_BYTES];
    size_t   length = 0;
    for (uint32_t i = 0; i < 100; i++) {
        _sim_run(&s, 1);
        if (s.out.advertise) {
            adverts++;
            length = db_control_advertisement(&s.control, 3000, buf);
        }
    }
    CHECK(adverts == 2, "advert: every 500 ms while not joined, %u in 1 s", adverts);
    CHECK(length == DB_CONTROL_ADVERTISEMENT_BYTES, "advert: %u bytes", (unsigned)length);
    CHECK(buf[0] == DB_PROTOCOL_DOTBOT_ADVERTISEMENT && buf[1] == 0xff, "advert: type and calibrated bitmask");
    uint16_t battery;
    memcpy(&battery, &buf[12], sizeof(battery));
    CHECK(battery == 3000, "advert: battery at offset 12, %u", battery);
    CHECK(buf[16] == ControlAuto, "advert: auto while a batch is active");
    int32_t encoder_left;
    memcpy(&encoder_left, &buf[17], sizeof(encoder_left));
    CHECK(encoder_left > 0 && (uint32_t)encoder_left < s.control.encoder_left, "advert: encoder counts since the previous advertisement, %d of %u", encoder_left, s.control.encoder_left);
    protocol_waypoints_report_t report;
    memcpy(&report, &buf[34], sizeof(report));
    CHECK(report.status == DB_WAYPOINTS_IN_PROGRESS && report.batch_id == 3, "advert: waypoint report in progress, batch 3");

    db_control_set_min_tx_interval(&s.control, 100000U);
    adverts = 0;
    for (uint32_t i = 0; i < 100; i++) {
        _sim_run(&s, 1);
        adverts += s.out.advertise;
    }
    CHECK(adverts == 5, "advert: at twice the 100 ms interval once joined, %u in 1 s", adverts);
}

static void test_fix_due(void) {
    db_control_t        control;
    db_control_input_t  in = { 0 };
    db_control_output_t out;
    db_control_init(&control, &db_control_default_conf);
    uint32_t due = 0;
    for (uint32_t i = 0; i < 30; i++) {
        if (db_control_fix_due(&control, 1)) {
            due++;
            CHECK(control.tick + 1 == 10 * due, "fix due: at tick %u", control.tick + 1);
        }
        db_control_tick(&control, &in, &out);
    }
    CHECK(due == 3, "fix due: every 10 ticks, %u in 30", due);
}

static void test_many_robots_independent(void) {
    static sim_t fleet[4];
    for (int i = 0; i < 4; i++) {
        _sim_init(&fleet[i], 1);
    }
    _send_batch(&fleet[1], 1);
    _send_velocity(&fleet[2], 200, 200);
    for (int i = 0; i < 4; i++) {
        _sim_run(&fleet[i], 20);
    }
    CHECK(fleet[0].control.drive_mode == DB_CONTROL_DRIVE_IDLE && fleet[3].control.drive_mode == DB_CONTROL_DRIVE_IDLE, "fleet: idle robots stay idle");
    CHECK(fleet[1].control.drive_mode == DB_CONTROL_DRIVE_WAYPOINT, "fleet: one robot drives waypoints");
    CHECK(fleet[2].control.drive_mode == DB_CONTROL_DRIVE_VELOCITY, "fleet: another drives by velocity");
}

int main(void) {
    test_batch_seeded();
    test_batch_from_boot();
    test_batch_dedup();
    test_max_speed();
    test_abort_reasons();
    test_raw_and_deadman();
    test_velocity_clamped();
    test_oversized_ignored();
    test_advertisement();
    test_fix_due();
    test_many_robots_independent();
    printf("dotbot_control: %d passed, %d failed\n", _passed, _failed);
    return _failed ? 1 : 0;
}
