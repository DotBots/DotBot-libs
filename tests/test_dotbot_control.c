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
    bool     airborne;  ///< held off the floor: the wheels turn, the robot and its fixes stay put
    float    air_x;     ///< photodiode fix while airborne, mm
    float    air_y;     ///< photodiode fix while airborne, mm
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

    float d      = p->airborne ? 0.0f : 0.5f * (dl + dr);
    float dtheta = p->airborne ? 0.0f : (dl - dr) / db_track_effective_mm(dl, dr) * 180.0f / (float)M_PI;
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
        p->fix_x = (uint32_t)lroundf(fmaxf(0, (p->airborne ? p->air_x : p->px[PLANT_FIX_AGE]) + nx));
        p->fix_y = (uint32_t)lroundf(fmaxf(0, (p->airborne ? p->air_y : p->py[PLANT_FIX_AGE]) + ny));
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
        db_control_seed(&s->control, 1000.0f, 1000.0f, 0.0f);
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

static void test_seed(void) {
    db_control_t        control;
    db_control_report_t report;
    db_control_init(&control, &db_control_default_conf);
    db_control_report(&control, &report);
    CHECK(report.direction == DB_CONTROL_DIRECTION_INVALID && report.axle_x == DB_AXLE_UNKNOWN, "seed: no heading and no axle from boot");

    db_control_seed(&control, 2000.0f, 1500.0f, 90.0f);
    db_control_report(&control, &report);
    CHECK(report.estimator_status == DB_POSE_ESTIMATOR_TRACKING, "seed: tracking at once, status %u", report.estimator_status);
    CHECK(report.direction == 90, "seed: advertises the heading, %d", report.direction);
    CHECK(report.axle_x == 2000 && report.axle_y == 1500, "seed: advertises the axle, %u %u", report.axle_x, report.axle_y);
    // Facing 90, body-forward is -x, so the photodiode sits the lever arm toward -x
    CHECK(report.sensor_x == (uint32_t)lroundf(2000.0f - DB_LH2_LEVER_ARM_EFFECTIVE) && report.sensor_y == 1500, "seed: photodiode the lever arm ahead, %u %u", report.sensor_x, report.sensor_y);
    CHECK(report.drive_mode == DB_CONTROL_DRIVE_IDLE && report.steering_state == DB_STEERING_IDLE, "seed: leaves the robot idle");

    sim_t s;
    _sim_init(&s, 0);
    _plant_init(&s.plant, 1000.0f, 1000.0f, 90.0f, 1.0f, 12345u);
    db_control_seed(&s.control, 1000.0f, 1000.0f, 90.0f);
    _send_batch(&s, 1);
    _sim_run(&s, 1500);
    CHECK(s.heading_tick == 10, "seed: a batch leaves NO_HEADING at the first steering step, tick %u", s.heading_tick);
    CHECK(s.arrived_tick > 0 && s.arrived_tick < 1000, "seed: arrives facing away from the first leg, took %u ticks", s.arrived_tick);
    CHECK(_miss(&s, 1500, 1000) < 30.0f + 5.0f, "seed: stops within the threshold, missed by %.1f", _miss(&s, 1500, 1000));
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
    CHECK(buf[0] == DB_PROTOCOL_DOTBOT_ADVERTISEMENT && buf[1] == 0xff && buf[2] == 0xff, "advert: type and calibrated station mask");
    uint16_t battery;
    memcpy(&battery, &buf[13], sizeof(battery));
    CHECK(battery == 3000, "advert: battery at offset 13, %u", battery);
    CHECK(buf[17] == ControlAuto, "advert: auto while a batch is active");
    int32_t encoder_left;
    memcpy(&encoder_left, &buf[18], sizeof(encoder_left));
    CHECK(encoder_left > 0 && (uint32_t)encoder_left < s.control.encoder_left, "advert: encoder counts since the previous advertisement, %d of %u", encoder_left, s.control.encoder_left);
    protocol_waypoints_report_t report;
    memcpy(&report, &buf[35], sizeof(report));
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

static void test_lifted_spins_once(void) {
    // Lifted mid-batch and held still, fixes jumping with the height: the
    // wheels drive on until the pose times out, then one spin in the air
    // fails the batch, braked
    sim_t s;
    _sim_init(&s, 1);
    _send_batch(&s, 1);
    _sim_run(&s, 300);
    s.plant.airborne = true;
    s.plant.air_x    = s.plant.px[0] + 40.0f;
    s.plant.air_y    = s.plant.py[0] + 40.0f;
    uint32_t bursts = 0, spin_ticks = 0;
    bool     moving = false;
    for (uint32_t i = 0; i < 1500; i++) {
        _sim_run(&s, 1);
        bool now = fabsf(s.plant.v_left) > 20.0f || fabsf(s.plant.v_right) > 20.0f;
        bursts += now && !moving;
        moving = now;
        spin_ticks += s.control.steering.state == DB_STEERING_NO_HEADING;
    }
    db_control_report_t report;
    db_control_report(&s.control, &report);
    CHECK(report.status == DB_WAYPOINTS_FAILED && report.reason == DB_STEERING_FAIL_NO_HEADING, "lifted: fails NO_HEADING, status %u reason %u", report.status, report.reason);
    CHECK(bursts == 2 && s.control.estimator.free_spins == 1, "lifted: the drive on, then one spin, bursts %u free spins %u", bursts, s.control.estimator.free_spins);
    CHECK(spin_ticks <= 30, "lifted: the spin is braked within two fixes of starting, %u ticks in NO_HEADING", spin_ticks);
    CHECK(report.brake_left && report.brake_right, "lifted: braked");
}

/// Runs ticks with two lines through each new fix staged before the tick,
/// both shifted by offset_mm
static void _run_with_lines(sim_t *s, uint32_t ticks, float offset_mm) {
    for (uint32_t i = 0; i < ticks; i++) {
        db_control_input_t in;
        _plant_step(&s->plant, &in);
        if (s->plant.tick % PLANT_FIX_EVERY == 0) {
            db_lh2_floor_line_t lines[2] = {
                { .nx = 1, .ny = 0, .d_mm = (float)in.fix_x + offset_mm, .var_mm2 = 4, .station = 3, .sweep = 0 },
                { .nx = 0, .ny = 1, .d_mm = (float)in.fix_y + offset_mm, .var_mm2 = 4, .station = 3, .sweep = 1 },
            };
            db_control_lines(&s->control, lines, 2);
        }
        db_control_tick(&s->control, &in, &s->out);
        _plant_apply(&s->plant, &s->out);
    }
}

static void test_lines_replace_the_fix(void) {
    sim_t s;
    _sim_init(&s, 1);
    _run_with_lines(&s, 200, 0);
    const db_pose_estimator_t *est = &s.control.estimator;
    CHECK(s.control.fusion.station[3].accepted == 40 && s.control.fusion.station[3].rejected == 0, "lines: both lines of every fix fused, %u accepted", s.control.fusion.station[3].accepted);
    CHECK(est->accepted == 0 && est->rejected == 0, "lines: the fix from the same sweeps is not applied as well, %u fixes", est->accepted);
    CHECK(est->status == DB_POSE_ESTIMATOR_TRACKING && s.control.has_position && s.control.line_count == 0, "lines: still tracking, the fix kept for the advertisement, nothing left staged");
    db_control_report_t report;
    db_control_report(&s.control, &report);
    CHECK(hypotf((float)report.sensor_x - s.plant.px[0], (float)report.sensor_y - s.plant.py[0]) < 3.0f, "lines: the advertised photodiode follows the truth");

    // Lines all outside the gate: the fix is gated as without lines
    _run_with_lines(&s, 50, 200.0f);
    CHECK(s.control.fusion.station[3].rejected == 10 && est->accepted == 5, "lines rejected: the fix is used instead, %u rejected lines, %u fixes", s.control.fusion.station[3].rejected, est->accepted);
}

/// One tick with lines through the fix, the fix and both lines moved by (dx, dy)
static void _tick_with_lines_offset(sim_t *s, float dx, float dy) {
    db_control_input_t in;
    _plant_step(&s->plant, &in);
    if (s->plant.tick % PLANT_FIX_EVERY == 0) {
        in.fix_x                     = (uint32_t)lroundf((float)in.fix_x + dx);
        in.fix_y                     = (uint32_t)lroundf((float)in.fix_y + dy);
        in.fix_sequence              = s->plant.fix_sequence + 1000000U;
        db_lh2_floor_line_t lines[2] = {
            { .nx = 1, .ny = 0, .d_mm = (float)in.fix_x, .var_mm2 = 4, .station = 3, .sweep = 0 },
            { .nx = 0, .ny = 1, .d_mm = (float)in.fix_y, .var_mm2 = 4, .station = 3, .sweep = 1 },
        };
        db_control_lines(&s->control, lines, 2);
    }
    db_control_tick(&s->control, &in, &s->out);
    _plant_apply(&s->plant, &s->out);
}

static void test_lines_keep_the_kidnap_check(void) {
    // Moved 300 mm along one line's direction: that line still agrees, the
    // other does not, so the fix is gated and the kidnap is seen
    sim_t s;
    _sim_init(&s, 1);
    _run_with_lines(&s, 200, 0);
    for (uint32_t i = 0; i < 100; i++) {
        _tick_with_lines_offset(&s, 0, 300.0f);
    }
    CHECK(s.control.estimator.kidnaps == 1 && s.control.estimator.status == DB_POSE_ESTIMATOR_SEEDING, "lines: a kidnap along one line's direction is still caught, %u kidnaps, status %u", s.control.estimator.kidnaps, s.control.estimator.status);
}

static void test_lines_clear_a_building_kidnap(void) {
    // A reflection on every tenth fix of a robot at rest is not a kidnap:
    // the good ticks between them confirm the pose
    sim_t s;
    _sim_init(&s, 1);
    _run_with_lines(&s, 200, 0);
    for (uint32_t i = 0; i < 2000; i++) {
        float off = ((s.plant.tick + 1) % (10 * PLANT_FIX_EVERY) == 0) ? 150.0f : 0.0f;
        _tick_with_lines_offset(&s, off, off);
    }
    CHECK(s.control.estimator.kidnaps == 0 && s.control.estimator.status == DB_POSE_ESTIMATOR_TRACKING, "lines: recurring reflections at rest cause no kidnap, %u kidnaps, status %u", s.control.estimator.kidnaps, s.control.estimator.status);
}

static void test_lines_ignored_while_seeding(void) {
    sim_t s;
    _sim_init(&s, 0);
    _send_velocity(&s, 150, 150);
    for (uint32_t i = 0; i < 100 && s.control.estimator.seeds == 0; i++) {
        _run_with_lines(&s, 1, 0);
    }
    CHECK(s.control.estimator.status == DB_POSE_ESTIMATOR_TRACKING && s.control.estimator.seeds == 1, "seeding: the fixes seed the pose as without lines");
    CHECK(s.control.fusion.station[3].accepted == 0 && s.control.fusion.station[3].rejected == 0, "seeding: lines until then are not counted");
}

/// Two lines through (x, y), crossing at 90 deg with normals turned by angle_deg
static void _cross(float x, float y, float angle_deg, uint8_t station, db_lh2_floor_line_t out[2]) {
    float a = angle_deg * (float)M_PI / 180.0f;
    float c = cosf(a), s = sinf(a);
    out[0] = (db_lh2_floor_line_t){ .nx = c, .ny = s, .d_mm = c * x + s * y, .var_mm2 = 1, .station = station, .sweep = 0 };
    out[1] = (db_lh2_floor_line_t){ .nx = -s, .ny = c, .d_mm = -s * x + c * y, .var_mm2 = 1, .station = station, .sweep = 1 };
}

/// Runs a robot at rest with no heading, staging with each new fix station 0's
/// lines through the truth and station 1's through the truth moved by
/// (bx, by) when both is set; the input's solve alternates between the two
/// stations. Returns the largest move of the advertised photodiode between
/// fixes, and its last offset from the truth.
static float _rest_no_heading(int both, float bx, float by, float *off_x, float *off_y) {
    sim_t s;
    _sim_init(&s, 0);
    float    worst = 0, prev_x = 0, prev_y = 0;
    uint32_t fixes = 0;
    for (uint32_t i = 0; i < 300; i++) {
        db_control_input_t in;
        _plant_step(&s.plant, &in);
        float tx = s.plant.px[0], ty = s.plant.py[0];
        if (s.plant.tick % PLANT_FIX_EVERY == 0) {
            db_lh2_floor_line_t lines[4];
            _cross(tx, ty, 20.0f, 0, &lines[0]);
            _cross(tx + bx, ty + by, 65.0f, 1, &lines[2]);
            db_control_lines(&s.control, lines, both ? 4 : 2);
            int pick_b = both && (s.plant.fix_sequence % 2);
            in.fix_x   = (uint32_t)lroundf(tx + (pick_b ? bx : 0));
            in.fix_y   = (uint32_t)lroundf(ty + (pick_b ? by : 0));
        }
        db_control_tick(&s.control, &in, &s.out);
        if (s.plant.tick % PLANT_FIX_EVERY == 0) {
            db_control_report_t report;
            db_control_report(&s.control, &report);
            if (fixes++ > 0) {
                worst = fmaxf(worst, hypotf((float)report.sensor_x - prev_x, (float)report.sensor_y - prev_y));
            }
            prev_x = (float)report.sensor_x;
            prev_y = (float)report.sensor_y;
            *off_x = prev_x - tx;
            *off_y = prev_y - ty;
        }
    }
    CHECK(s.control.estimator.status != DB_POSE_ESTIMATOR_TRACKING, "no heading: the robot at rest never tracks");
    return worst;
}

static void test_fix_from_every_station_before_heading(void) {
    // The stations disagree by 6 mm along x; the secure side's solve hops between them
    float ox, oy;
    float step = _rest_no_heading(1, 6.0f, 0, &ox, &oy);
    CHECK(step <= 1.0f, "no heading, two stations: no hop between fixes, largest step %.1f mm", step);
    CHECK(fabsf(ox - 3.0f) <= 1.0f && fabsf(oy) <= 1.0f, "no heading, two stations: the fix sits midway, (%.1f, %.1f) mm from station 0", ox, oy);
    step = _rest_no_heading(0, 0, 0, &ox, &oy);
    CHECK(step <= 1.0f && hypotf(ox, oy) <= 1.0f, "no heading, one station: its intersection, (%.1f, %.1f) mm off, step %.1f", ox, oy, step);
}

/// Runs a robot at rest with no heading, staging with each new fix two
/// near-parallel lines through the input's solve moved by off_mm along x
static void _near_parallel(sim_t *s, float off_mm) {
    _sim_init(s, 0);
    for (uint32_t i = 0; i < 20; i++) {
        db_control_input_t in;
        _plant_step(&s->plant, &in);
        if (s->plant.tick % PLANT_FIX_EVERY == 0) {
            float               x = (float)in.fix_x + off_mm, y = (float)in.fix_y;
            db_lh2_floor_line_t lines[2] = {
                { .nx = 1, .ny = 0, .d_mm = x, .var_mm2 = 1, .station = 0, .sweep = 0 },
                { .nx = 1, .ny = 0.01f, .d_mm = x + 0.01f * y, .var_mm2 = 1, .station = 1, .sweep = 0 },
            };
            db_control_lines(&s->control, lines, 2);
        }
        db_control_tick(&s->control, &in, &s->out);
    }
}

static void test_fix_falls_back_without_a_crossing(void) {
    sim_t s;
    _near_parallel(&s, 0);
    CHECK(s.control.has_position && s.control.position_x == s.plant.fix_x && s.control.position_y == s.plant.fix_y, "near-parallel lines: the input's solve is the fix, (%u, %u)", s.control.position_x, s.control.position_y);
    _near_parallel(&s, 20.0f);
    CHECK(!s.control.has_position && !s.control.last_fusion.fix_valid, "near-parallel lines 20 mm off the input's solve: no fix");
}

static void test_fix_leaves_out_a_rejected_station(void) {
    // Tracking at rest; station 1's lines are 40 mm off and gated out
    sim_t s;
    _sim_init(&s, 1);
    s.plant.noise_mm = 0;
    for (uint32_t i = 0; i < 300; i++) {
        db_control_input_t in;
        _plant_step(&s.plant, &in);
        if (s.plant.tick % PLANT_FIX_EVERY == 0) {
            db_lh2_floor_line_t lines[4];
            _cross(s.plant.px[PLANT_FIX_AGE], s.plant.py[PLANT_FIX_AGE], 20.0f, 0, &lines[0]);
            _cross(s.plant.px[PLANT_FIX_AGE] + 40.0f, s.plant.py[PLANT_FIX_AGE], 65.0f, 1, &lines[2]);
            db_control_lines(&s.control, lines, 4);
        }
        db_control_tick(&s.control, &in, &s.out);
    }
    const db_pose_estimator_t *est = &s.control.estimator;
    CHECK(s.control.fusion.station[1].rejected > 0 && s.control.fusion.station[1].accepted == 0, "bad station: its lines are gated out, %u rejected", s.control.fusion.station[1].rejected);
    CHECK(est->kidnaps == 0 && est->status == DB_POSE_ESTIMATOR_TRACKING && est->rejected == 0, "bad station: the fix from the good one is no kidnap, %u kidnaps, %u fixes rejected", est->kidnaps, est->rejected);
    CHECK(hypotf((float)s.control.position_x - s.plant.px[PLANT_FIX_AGE], (float)s.control.position_y - s.plant.py[PLANT_FIX_AGE]) <= 1.0f, "bad station: the fix is the good station's point");

    // The record names each line's station, sweep, result and innovation
    const db_control_fusion_record_t *r = &s.control.last_fusion;
    CHECK(r->count == 4 && r->station_sweep[0] == 0 && r->station_sweep[1] == 1 && r->station_sweep[2] == 2 && r->station_sweep[3] == 3, "record: four lines, station << 1 | sweep, count %u", r->count);
    CHECK(r->result[0] == DB_POSE_ESTIMATOR_ACCEPTED && r->result[1] == DB_POSE_ESTIMATOR_ACCEPTED && r->result[2] == DB_POSE_ESTIMATOR_REJECTED && r->result[3] == DB_POSE_ESTIMATOR_REJECTED, "record: station 0 accepted, station 1 rejected");
    CHECK(fabsf(r->innovation_mm[0]) < 1.0f && fabsf(r->innovation_mm[2]) > 10.0f && isfinite(r->innovation_mm[3]), "record: innovations %.1f and %.1f mm", r->innovation_mm[0], r->innovation_mm[2]);
    CHECK(r->fix_valid && r->fix_from_lines && lroundf(r->fix_x) == (long)s.control.position_x && lroundf(r->fix_y) == (long)s.control.position_y, "record: the fix is the accepted lines' point");
}

static void test_fusion_record_before_heading_and_on_fallback(void) {
    sim_t s;
    _sim_init(&s, 0);
    for (uint32_t i = 0; i < 20; i++) {
        db_control_input_t in;
        _plant_step(&s.plant, &in);
        if (s.plant.tick % PLANT_FIX_EVERY == 0) {
            float               x = (float)in.fix_x, y = (float)in.fix_y;
            db_lh2_floor_line_t lines[2] = {
                { .nx = 1, .ny = 0, .d_mm = x, .var_mm2 = 1, .station = 0, .sweep = 0 },
                { .nx = 1, .ny = 0.01f, .d_mm = x + 0.01f * y, .var_mm2 = 1, .station = 5, .sweep = 1 },
            };
            db_control_lines(&s.control, lines, 2);
        }
        db_control_tick(&s.control, &in, &s.out);
    }
    const db_control_fusion_record_t *r = &s.control.last_fusion;
    CHECK(r->count == 2 && r->station_sweep[1] == 11, "fallback record: two lines, the second station 5 sweep 1");
    CHECK(r->result[0] == DB_POSE_ESTIMATOR_REJECTED && isnan(r->innovation_mm[0]) && isnan(r->innovation_mm[1]), "fallback record: not gated with no heading, innovations NaN");
    CHECK(r->fix_valid && !r->fix_from_lines && r->fix_x == (float)s.plant.fix_x, "fallback record: the input's solve, flagged as such");

    db_lh2_floor_line_t lines[2];
    _cross(200.0f, 300.0f, 20.0f, 1, lines);
    db_control_input_t in;
    _plant_step(&s.plant, &in);
    db_control_lines(&s.control, lines, 2);
    db_control_tick(&s.control, &in, &s.out);
    CHECK(r->fix_from_lines && fabsf(r->fix_x - 200.0f) < 0.01f && fabsf(r->fix_y - 300.0f) < 0.01f, "no heading record: the lines' point, (%.2f, %.2f)", r->fix_x, r->fix_y);

    _plant_step(&s.plant, &in);
    db_control_tick(&s.control, &in, &s.out);
    CHECK(r->count == 2 && r->fix_from_lines && fabsf(r->fix_x - 200.0f) < 0.01f, "a tick with no lines leaves the record as it was");
}

/// A 60 mm drive for a heading, then a stand under two stations 3.7 mm
/// apart, station 0 dark for 14 s of it; the heading change over the dark
/// window, deg. Without lines, the input's solve carries the station in view.
static float _dark_at_rest_swing(int with_lines) {
    sim_t s;
    _sim_init(&s, 0);
    s.plant.noise_mm = 0;
    float    start_x = s.plant.x, start_y = s.plant.y, rest_deg = 0;
    uint32_t stop = 0;
    for (uint32_t i = 0; i < 3000; i++) {
        if (stop == 0 && i % 20 == 0) {
            _send_velocity(&s, 150, 150);
        }
        if (stop == 0 && hypotf(s.plant.x - start_x, s.plant.y - start_y) >= 60.0f) {
            _send_velocity(&s, 0, 0);
            stop = i;
        }
        bool dark = stop > 0 && i >= stop + 1000 && i < stop + 2400;
        if (stop > 0 && i == stop + 1000) {
            rest_deg = s.control.estimator.theta * 180.0f / (float)M_PI;
        }
        db_control_input_t in;
        _plant_step(&s.plant, &in);
        if (s.plant.tick % PLANT_FIX_EVERY == 0) {
            float tx = s.plant.px[PLANT_FIX_AGE], ty = s.plant.py[PLANT_FIX_AGE];
            if (with_lines) {
                db_lh2_floor_line_t lines[4];
                _cross(tx, ty, 20.0f, 0, &lines[0]);
                _cross(tx - 2.1f, ty - 3.0f, 65.0f, 1, &lines[2]);
                db_control_lines(&s.control, dark ? &lines[2] : lines, dark ? 2 : 4);
            } else if (dark) {
                in.fix_x = (uint32_t)lroundf(tx - 2.1f);
                in.fix_y = (uint32_t)lroundf(ty - 3.0f);
            }
        }
        db_control_tick(&s.control, &in, &s.out);
        _plant_apply(&s.plant, &s.out);
        if (stop > 0 && i == stop + 2400) {
            break;
        }
    }
    CHECK(stop > 0 && s.control.estimator.status == DB_POSE_ESTIMATOR_TRACKING, "dark at rest: tracking after the drive");
    return fabsf(s.control.estimator.theta * 180.0f / (float)M_PI - rest_deg);
}

static void test_station_dark_at_rest_keeps_the_heading(void) {
    float swing = _dark_at_rest_swing(1);
    CHECK(swing < 0.5f, "dark at rest, lines: the heading holds while they shift, swung %.2f deg", swing);
    swing = _dark_at_rest_swing(0);
    CHECK(swing < 0.5f, "dark at rest, fixes: the heading holds while they shift, swung %.2f deg", swing);
}

/// Tracking at rest under stations 0 and 1, station 1 dark for gap_ticks from
/// tick 1000, then back with its lines 3 mm off; station 1's lines fused in
/// the window_ticks after it is back
static uint32_t _station_back_fused(uint32_t gap_ticks, uint32_t window_ticks) {
    sim_t s;
    _sim_init(&s, 1);
    s.plant.noise_mm = 0;
    uint32_t back = 1000 + gap_ticks, accepted_at_back = 0;
    for (uint32_t i = 0; i < back + window_ticks; i++) {
        db_control_input_t in;
        _plant_step(&s.plant, &in);
        if (s.plant.tick % PLANT_FIX_EVERY == 0) {
            float               tx = s.plant.px[PLANT_FIX_AGE], ty = s.plant.py[PLANT_FIX_AGE];
            db_lh2_floor_line_t lines[4];
            _cross(tx, ty, 20.0f, 0, &lines[0]);
            _cross(tx + (i >= back ? 3.0f : 0), ty, 65.0f, 1, &lines[2]);
            bool dark = i >= 1000 && i < back;
            db_control_lines(&s.control, lines, dark ? 2 : 4);
        }
        db_control_tick(&s.control, &in, &s.out);
        if (i + 1 == back) {
            accepted_at_back = s.control.fusion.station[1].accepted;
        }
    }
    CHECK(s.control.estimator.status == DB_POSE_ESTIMATOR_TRACKING, "station back: still tracking");
    return s.control.fusion.station[1].accepted - accepted_at_back;
}

static void test_station_back_on_air_settles(void) {
    uint32_t fused = _station_back_fused(1000, DB_CONTROL_STATION_SETTLE_TICKS);
    CHECK(fused == 0, "back after 10 s: its lines left out while it settles, %u fused", fused);
    fused = _station_back_fused(1000, DB_CONTROL_STATION_SETTLE_TICKS + 100);
    CHECK(fused > 0, "back after 10 s: its lines fused once settled, %u fused", fused);
    fused = _station_back_fused(30, 200);
    CHECK(fused > 0, "back after 300 ms: not an absence, %u lines fused", fused);
}

/// A robot at rest where station 1's sweep 0 line is 20 mm off along its
/// normal on a random 57 % of reads, the photodiode moved by (dx, dy), and
/// station 0 in a read with probability both_pct %, and absent for the first
/// 6 of every drop_every reads when that is set; the input's solve is
/// station 1's pair. Returns the advertised photodiode's largest distance from
/// the truth over the fixes after the first skip ones.
static float _bimodal_at_rest(sim_t *s, uint32_t fixes, uint32_t skip, uint32_t both_pct, uint32_t drop_every, float dx, float dy) {
    float worst = 0;
    for (uint32_t n = 0; n < fixes;) {
        db_control_input_t in;
        _plant_step(&s->plant, &in);
        bool  fix = s->plant.tick % PLANT_FIX_EVERY == 0;
        float tx  = s->plant.px[PLANT_FIX_AGE] + dx;
        float ty  = s->plant.py[PLANT_FIX_AGE] + dy;
        if (fix) {
            db_lh2_floor_line_t lines[4];
            _cross(tx, ty, 20.0f, 0, &lines[0]);
            _cross(tx, ty, 65.0f, 1, &lines[2]);
            for (unsigned i = 0; i < 4; i++) {
                lines[i].var_mm2 = 4;
                lines[i].d_mm += _gauss(&s->plant);
            }
            if (_uniform(&s->plant) < 0.57f) {
                lines[2].d_mm += 20.0f;
            }
            bool  both = _uniform(&s->plant) * 100.0f < (float)both_pct && !(drop_every && n % drop_every < 6);
            float sx = tx, sy = ty;
            db_lh2_fusion_intersect(&lines[2], 2, &sx, &sy);
            in.fix_x = (uint32_t)lroundf(sx);
            in.fix_y = (uint32_t)lroundf(sy);
            db_control_lines(&s->control, both ? lines : &lines[2], both ? 4 : 2);
        }
        db_control_tick(&s->control, &in, &s->out);
        if (fix) {
            db_control_report_t report;
            db_control_report(&s->control, &report);
            if (n++ >= skip) {
                worst = fmaxf(worst, hypotf((float)report.sensor_x - tx, (float)report.sensor_y - ty));
            }
        }
    }
    return worst;
}

static void test_bimodal_line_keeps_tracking(void) {
    // On a read with station 1 alone, its good sweep is accepted and the bad
    // one rejected: one line fixes no point, and the solve is the bad one
    sim_t s;
    _sim_init(&s, 1);
    s.plant.noise_mm               = 0;
    float                      off = _bimodal_at_rest(&s, 1000, 0, 70, 0, 0, 0);
    const db_pose_estimator_t *est = &s.control.estimator;
    CHECK(est->status == DB_POSE_ESTIMATOR_TRACKING && est->kidnaps == 0, "bimodal line, tracking: still tracking over 1000 fixes, %u kidnaps, status %u", est->kidnaps, est->status);
    CHECK(off <= 1.5f, "bimodal line, tracking: the advertised photodiode within 1.5 mm, %.1f mm off", off);
}

static void test_bimodal_line_untracked_holds_still(void) {
    // Never driven, so never tracking: the advertisement is each read's fix
    sim_t s;
    _sim_init(&s, 0);
    s.plant.noise_mm = 0;
    float off        = _bimodal_at_rest(&s, 1000, 10, 90, 0, 0, 0);
    CHECK(s.control.estimator.status != DB_POSE_ESTIMATOR_TRACKING, "bimodal line, untracked: never tracks at rest");
    CHECK(off <= 5.0f, "bimodal line, untracked: the advertised photodiode within 5 mm after the first second, %.1f mm off", off);

    // As on the bench: station 0 off air for 0.6 s every 6 s, then settling
    _sim_init(&s, 0);
    s.plant.noise_mm = 0;
    off              = _bimodal_at_rest(&s, 1000, 10, 100, 60, 0, 0);
    CHECK(off <= 5.0f, "bimodal line, untracked, station 0 dropping out: within 5 mm through its settling, %.1f mm off", off);
}

static void test_untracked_at_rest_follows_a_move_by_hand(void) {
    // Carried, wheels still, to where station 1 alone is in view: its fixes
    // are taken once the last checked fix is old enough
    sim_t s;
    _sim_init(&s, 0);
    s.plant.noise_mm = 0;
    _bimodal_at_rest(&s, 100, 0, 100, 0, 0, 0);
    _bimodal_at_rest(&s, 50, 0, 0, 0, 300.0f, -200.0f);
    float tx = s.plant.px[PLANT_FIX_AGE] + 300.0f, ty = s.plant.py[PLANT_FIX_AGE] - 200.0f;
    float d = hypotf((float)s.control.position_x - tx, (float)s.control.position_y - ty);
    CHECK(d < 25.0f, "untracked, moved 360 mm by hand: the fix follows within 5 s, %.1f mm off", d);
}

static void test_bimodal_line_still_sees_a_kidnap(void) {
    sim_t s;
    _sim_init(&s, 1);
    s.plant.noise_mm = 0;
    _bimodal_at_rest(&s, 200, 0, 70, 0, 0, 0);
    _bimodal_at_rest(&s, 20, 0, 70, 0, 300.0f, -200.0f);
    const db_pose_estimator_t *est = &s.control.estimator;
    CHECK(est->kidnaps == 1 && est->status == DB_POSE_ESTIMATOR_SEEDING, "bimodal line, moved 360 mm: the kidnap is seen within 2 s, %u kidnaps, status %u", est->kidnaps, est->status);
}

int main(void) {
    test_batch_seeded();
    test_seed();
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
    test_lifted_spins_once();
    test_lines_replace_the_fix();
    test_lines_ignored_while_seeding();
    test_lines_keep_the_kidnap_check();
    test_lines_clear_a_building_kidnap();
    test_fix_from_every_station_before_heading();
    test_fix_falls_back_without_a_crossing();
    test_fix_leaves_out_a_rejected_station();
    test_fusion_record_before_heading_and_on_fallback();
    test_station_dark_at_rest_keeps_the_heading();
    test_station_back_on_air_settles();
    test_bimodal_line_keeps_tracking();
    test_bimodal_line_untracked_holds_still();
    test_bimodal_line_still_sees_a_kidnap();
    test_untracked_at_rest_follows_a_move_by_hand();
    printf("dotbot_control: %d passed, %d failed\n", _passed, _failed);
    return _failed ? 1 : 0;
}
