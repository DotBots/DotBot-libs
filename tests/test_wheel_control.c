/**
 * @file
 * @brief  Host test of drv/wheel_control against a modelled wheel
 *
 * The plant is a model with guessed constants (stiction, a running line, a
 * first-order lag), not the robot: these checks prove the controller's logic,
 * not its tuning.
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "geometry.h"
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

//=========================== plant ============================================

#define PLANT_SUBSTEPS (10)

typedef struct {
    float u_static;   ///< duty needed to start a stopped wheel
    float u_run;      ///< duty of the running line at zero speed
    float k_run;      ///< duty per mm/s on the running line
    float tau_s;      ///< speed lag
    float speed;      ///< mm/s
    float travel_mm;  ///< mm not yet turned into whole counts
    int   blocked;    ///< wheel held still
} plant_t;

static void _plant_init(plant_t *p) {
    p->u_static  = 44.0f;
    p->u_run     = 38.0f;
    p->k_run     = 0.13f;
    p->tau_s     = 0.1f;
    p->speed     = 0;
    p->travel_mm = 0;
    p->blocked   = 0;
}

/// Advance one 10 ms tick under a duty, return the whole counts produced
static int32_t _plant_step(plant_t *p, int8_t pwm, uint32_t ticks) {
    float dt = (float)ticks * (DB_WHEEL_CONTROL_TICK_MS / 1000.0f) / PLANT_SUBSTEPS;
    for (int i = 0; i < PLANT_SUBSTEPS; i++) {
        float target = 0;
        float u      = fabsf((float)pwm);
        if (p->blocked) {
            p->speed = 0;
            continue;
        }
        if (p->speed == 0 && u < p->u_static) {
            continue;
        }
        if (u > p->u_run) {
            target = copysignf((u - p->u_run) / p->k_run, (float)pwm);
        }
        p->speed += (target - p->speed) * dt / p->tau_s;
        if (fabsf(p->speed) < 1.0f && u < p->u_static) {
            p->speed = 0;
        }
        p->travel_mm += p->speed * dt;
    }
    int32_t counts = (int32_t)(p->travel_mm / DB_MM_PER_COUNT);
    p->travel_mm -= (float)counts * DB_MM_PER_COUNT;
    return counts;
}

//=========================== fixtures =========================================

static const db_wheel_control_conf_t _conf = {
    .kp                = 0.5f,
    .ki                = 5.0f,
    .u_breakaway       = 44.0f,
    .kick_ramp         = 0.5f,
    .u_run             = 36.0f,
    .k_run             = 0.13f,
    .i_zone            = 50.0f,
    .pwm_max           = 75.0f,
    .pwm_slew_per_tick = 20.0f,
};

typedef struct {
    float peak;         ///< highest measured speed, mm/s
    int   ticks_to_90;  ///< first tick at 90 % of the setpoint, -1 if never
    float mean_last_s;  ///< mean measured speed over the last second, mm/s
    float max_slew;     ///< largest output change in one step
    int   min_pwm;      ///< lowest output
    int   max_pwm;      ///< highest output
    float plant_speed;  ///< plant speed at the end
} run_t;

/// Run the loop against the plant for a number of ticks at one setpoint
static run_t _run(db_wheel_control_t *w, plant_t *p, float setpoint, int ticks) {
    run_t   r         = { .peak = -1e9f, .ticks_to_90 = -1, .min_pwm = 127, .max_pwm = -128 };
    int8_t  pwm       = (int8_t)w->pwm;
    float   sum       = 0;
    int32_t counts    = 0;
    float   direction = (setpoint >= 0) ? 1.0f : -1.0f;
    db_wheel_control_set_setpoint(w, setpoint);
    for (int t = 0; t < ticks; t++) {
        counts                = _plant_step(p, pwm, 1);
        int8_t next           = db_wheel_control_step(w, counts, 1);
        float  slew           = fabsf((float)next - (float)pwm);
        pwm                   = next;
        float signed_measured = w->measured * direction;
        if (slew > r.max_slew) {
            r.max_slew = slew;
        }
        if (pwm < r.min_pwm) {
            r.min_pwm = pwm;
        }
        if (pwm > r.max_pwm) {
            r.max_pwm = pwm;
        }
        if (signed_measured > r.peak) {
            r.peak = signed_measured;
        }
        if (r.ticks_to_90 < 0 && signed_measured >= 0.9f * fabsf(setpoint)) {
            r.ticks_to_90 = t + 1;
        }
        if (t >= ticks - 100) {
            sum += w->measured;
        }
    }
    r.mean_last_s = sum / 100.0f;
    r.plant_speed = p->speed;
    return r;
}

//=========================== tests ============================================

static void test_zero_setpoint_no_creep(void) {
    db_wheel_control_t w;
    plant_t            p;
    db_wheel_control_init(&w, &_conf);
    _plant_init(&p);
    run_t r = _run(&w, &p, 0, 300);
    CHECK(r.min_pwm == 0 && r.max_pwm == 0, "zero setpoint must output zero, got [%d, %d]", r.min_pwm, r.max_pwm);
    CHECK(p.speed == 0, "zero setpoint must leave the wheel still, speed %f", p.speed);
}

static void test_step_from_rest(float setpoint) {
    db_wheel_control_t w;
    plant_t            p;
    db_wheel_control_init(&w, &_conf);
    _plant_init(&p);
    run_t r = _run(&w, &p, setpoint, 300);
    float s = fabsf(setpoint);
    CHECK(r.ticks_to_90 > 0 && r.ticks_to_90 <= 20, "%.0f mm/s from rest: 90%% at tick %d, want <= 20 (200 ms)", setpoint, r.ticks_to_90);
    CHECK(r.peak <= 1.15f * s, "%.0f mm/s from rest: peak %.1f, want <= 115%%", setpoint, r.peak);
    CHECK(fabsf(r.mean_last_s - setpoint) <= 0.05f * s, "%.0f mm/s from rest: steady %.1f, want within 5%%", setpoint, r.mean_last_s);
    CHECK(r.max_slew <= _conf.pwm_slew_per_tick, "%.0f mm/s from rest: slew %.1f over the limit", setpoint, r.max_slew);
    CHECK(r.max_pwm <= _conf.pwm_max && r.min_pwm >= -_conf.pwm_max, "%.0f mm/s from rest: output [%d, %d] outside the saturation", setpoint, r.min_pwm, r.max_pwm);
}

static void test_step_up(void) {
    db_wheel_control_t w;
    plant_t            p;
    db_wheel_control_init(&w, &_conf);
    _plant_init(&p);
    _run(&w, &p, 150, 300);
    run_t r = _run(&w, &p, 250, 300);
    CHECK(r.ticks_to_90 > 0 && r.ticks_to_90 <= 20, "150 to 250: 90%% at tick %d, want <= 20", r.ticks_to_90);
    CHECK(fabsf(r.mean_last_s - 250) <= 12.5f, "150 to 250: steady %.1f, want within 5%%", r.mean_last_s);
}

static void test_feedforward_sign(void) {
    db_wheel_control_t w;
    db_wheel_control_init(&w, &_conf);
    db_wheel_control_set_setpoint(&w, 150);
    int8_t fwd  = db_wheel_control_step(&w, 0, 1);
    float  kick = fmaxf(_conf.u_breakaway, _conf.u_run + _conf.k_run * 150);
    CHECK(fwd > 0 && w.ff == kick, "a stalled forward step kicks forward, got pwm %d ff %.1f", fwd, w.ff);
    db_wheel_control_reset(&w);
    db_wheel_control_set_setpoint(&w, -150);
    int8_t back = db_wheel_control_step(&w, 0, 1);
    CHECK(back < 0 && w.ff == -kick, "a stalled backward step kicks backward, got pwm %d ff %.1f", back, w.ff);
    db_wheel_control_step(&w, -3, 1);
    CHECK(fabsf(w.ff + (_conf.u_run + _conf.k_run * 150)) < 1e-4f, "a turning wheel takes the running line, got ff %.2f", w.ff);
}

static void test_stop_is_immediate(void) {
    db_wheel_control_t w;
    plant_t            p;
    db_wheel_control_init(&w, &_conf);
    _plant_init(&p);
    _run(&w, &p, 300, 200);
    db_wheel_control_set_setpoint(&w, 0);
    int8_t pwm = db_wheel_control_step(&w, 5, 1);
    CHECK(pwm == 0 && w.integral == 0, "a zero setpoint stops at once and clears the integral, got pwm %d integral %.2f", pwm, w.integral);
}

/// Ask for slightly more than the wheel can do, then for something it can: the loop
/// must settle no slower than a P-only loop on the same scenario, which cannot
/// wind up by construction.
static int _recovery_ticks(const db_wheel_control_conf_t *conf) {
    db_wheel_control_t w;
    plant_t            p;
    db_wheel_control_init(&w, conf);
    _plant_init(&p);
    // Just out of reach: the error stays inside the integral zone while the
    // output sits at the saturation, which is where an unbounded integral grows
    _run(&w, &p, 300, 500);
    db_wheel_control_set_setpoint(&w, 150);
    int8_t pwm = (int8_t)w.pwm;
    for (int t = 0; t < 500; t++) {
        int32_t counts = _plant_step(&p, pwm, 1);
        pwm            = db_wheel_control_step(&w, counts, 1);
        if (p.speed < 1.2f * 150) {
            return t;
        }
    }
    return 1000;
}

static void test_no_windup(void) {
    db_wheel_control_conf_t p_only = _conf;
    p_only.ki                      = 0;
    int with_i                     = _recovery_ticks(&_conf);
    int without_i                  = _recovery_ticks(&p_only);
    CHECK(with_i < 1000, "the loop comes down from saturation, took %d ticks", with_i);
    CHECK(with_i <= without_i + 5, "down from saturation: %d ticks with the integral, %d without; the integral wound up", with_i, without_i);
}

static void test_blocked_wheel_bounded(void) {
    db_wheel_control_t w;
    plant_t            p;
    db_wheel_control_init(&w, &_conf);
    _plant_init(&p);
    p.blocked = 1;
    run_t r   = _run(&w, &p, 400, 500);
    CHECK(r.max_pwm <= _conf.pwm_max, "a held wheel stays within the saturation, max %d", r.max_pwm);
    CHECK(w.ff == _conf.pwm_max, "a held wheel's kick ramps up to the saturation, ff %.1f", w.ff);
    CHECK(_conf.ki * w.integral <= _conf.pwm_max - w.ff + 1e-3f, "a held wheel's integral is bounded, holds %.1f duty", _conf.ki * w.integral);
}

static void test_reversal(void) {
    db_wheel_control_t w;
    plant_t            p;
    db_wheel_control_init(&w, &_conf);
    _plant_init(&p);
    _run(&w, &p, 250, 200);
    run_t r = _run(&w, &p, -250, 300);
    CHECK(fabsf(r.mean_last_s + 250) <= 12.5f, "250 to -250: steady %.1f, want within 5%%", r.mean_last_s);
    CHECK(r.max_slew <= _conf.pwm_slew_per_tick, "250 to -250: slew %.1f over the limit", r.max_slew);
}

static void test_elapsed_ticks(void) {
    db_wheel_control_t w;
    db_wheel_control_init(&w, &_conf);
    db_wheel_control_set_setpoint(&w, 100);
    db_wheel_control_step(&w, 10, 2);
    float expected = 10 * DB_MM_PER_COUNT / 0.020f;
    CHECK(fabsf(w.measured - expected) < 1e-3f, "two elapsed ticks divide by 20 ms: %.3f, want %.3f", w.measured, expected);
    float  before = w.pwm;
    int8_t again  = db_wheel_control_step(&w, 0, 0);
    CHECK(again == (int8_t)before && !isnan(w.pwm), "zero elapsed ticks returns the previous output, got %d want %d", again, (int8_t)before);
}

static void test_counts(void) {
    CHECK(db_wheel_control_counts(10, 2) == 14, "forward doubles add two steps each, got %d", db_wheel_control_counts(10, 2));
    CHECK(db_wheel_control_counts(-10, 2) == -14, "backward doubles subtract two steps each, got %d", db_wheel_control_counts(-10, 2));
    CHECK(db_wheel_control_counts(0, 3) == 0, "doubles with no direction are dropped, got %d", db_wheel_control_counts(0, 3));
    CHECK(db_wheel_control_counts(-7, 0) == -7, "no doubles leaves the accumulator, got %d", db_wheel_control_counts(-7, 0));
}

static void test_twist(void) {
    float           l, r;
    db_body_twist_t straight = { .v_mm_s = 120, .omega_deg_s = 0 };
    db_wheel_control_from_twist(&straight, &l, &r);
    CHECK(l == 120 && r == 120, "no turn rate drives both wheels equally, got %.1f %.1f", l, r);
    db_body_twist_t clockwise = { .v_mm_s = 0, .omega_deg_s = 90 };
    db_wheel_control_from_twist(&clockwise, &l, &r);
    float half = (float)M_PI / 2.0f * DB_TRACK / 2.0f;
    CHECK(fabsf(l - half) < 1e-3f && fabsf(r + half) < 1e-3f, "clockwise speeds the left wheel up: got %.2f %.2f, want %.2f %.2f", l, r, half, -half);
}

static void test_integral_zone(void) {
    db_wheel_control_t w;
    db_wheel_control_init(&w, &_conf);
    db_wheel_control_set_setpoint(&w, 200);
    db_wheel_control_step(&w, 3, 1);
    CHECK(w.integral == 0, "far below the setpoint the integral holds still, holds %.2f", w.integral);
    db_wheel_control_step(&w, 19, 1);
    db_wheel_control_step(&w, 19, 1);
    CHECK(w.integral != 0, "inside the zone the integral accumulates, holds %.2f", w.integral);
}

static void test_sign_change_clears_integral(void) {
    db_wheel_control_t w;
    plant_t            p;
    db_wheel_control_init(&w, &_conf);
    _plant_init(&p);
    _run(&w, &p, 200, 100);
    db_wheel_control_set_setpoint(&w, -200);
    CHECK(w.integral == 0, "a setpoint sign change clears the integral, holds %.2f", w.integral);
}

static void test_period_two_rejected(void) {
    // Counts alternating every step around a steady mean, as a wheel limit
    // cycling through its gearbox reads: the output must not follow them
    db_wheel_control_conf_t conf = _conf;
    conf.ki                      = 0;
    db_wheel_control_t w;
    db_wheel_control_init(&w, &conf);
    db_wheel_control_set_setpoint(&w, 24 * DB_MM_PER_COUNT / 0.010f);
    int8_t pwm   = 0;
    float  swing = 0;
    for (int t = 0; t < 60; t++) {
        int8_t next = db_wheel_control_step(&w, (t % 2) ? 18 : 30, 1);
        if (t >= 40) {
            swing += fabsf((float)next - (float)pwm);
        }
        pwm = next;
    }
    CHECK(swing / 20 <= 1.0f, "counts alternating 30/18: output changes %.1f per step, want <= 1", swing / 20);
}

static void test_stall_start_pushes(void) {
    // A wheel still at rest takes the P term on the whole setpoint, not the kick alone
    db_wheel_control_t w;
    db_wheel_control_init(&w, &_conf);
    db_wheel_control_set_setpoint(&w, 150);
    int8_t pwm = 0;
    for (int t = 0; t < 5; t++) {
        pwm = db_wheel_control_step(&w, 0, 1);
    }
    CHECK(pwm == _conf.pwm_max, "a stalled step from rest pushes to the saturation, got %d, kick %.1f", pwm, w.ff);
}

static void test_brakes_through_dead_zone(void) {
    // Over its setpoint by less than the feedforward, a wheel would get a duty
    // between 0 and u_run, where it only coasts: it must be driven backward instead
    db_wheel_control_t w;
    db_wheel_control_init(&w, &_conf);
    db_wheel_control_set_setpoint(&w, 300);
    int32_t counts = (int32_t)roundf(300 * 0.010f / DB_MM_PER_COUNT);
    for (int t = 0; t < 50; t++) {
        db_wheel_control_step(&w, counts, 1);
    }
    db_wheel_control_set_setpoint(&w, 200);
    int8_t pwm = 0;
    for (int t = 0; t < 10; t++) {
        pwm = db_wheel_control_step(&w, counts, 1);
    }
    CHECK(pwm <= -_conf.u_run / 2, "a wheel 100 mm/s over its setpoint brakes, got duty %d", pwm);
}

static void test_coasts_near_setpoint(void) {
    // Slightly over a slow setpoint the command falls inside the dead zone; it
    // must not be turned into braking, or a wheel the body carries chatters
    db_wheel_control_t w;
    db_wheel_control_init(&w, &_conf);
    db_wheel_control_set_setpoint(&w, 30);
    int32_t counts = (int32_t)roundf(50 * 0.010f / DB_MM_PER_COUNT);
    int8_t  pwm    = 0;
    for (int t = 0; t < 3; t++) {
        pwm = db_wheel_control_step(&w, counts, 1);
    }
    CHECK(pwm >= 0, "20 mm/s over a 30 mm/s setpoint coasts rather than brakes, got duty %d", pwm);
}

int main(void) {
    test_zero_setpoint_no_creep();
    test_step_from_rest(200);
    test_step_from_rest(-200);
    test_step_from_rest(100);
    test_step_up();
    test_feedforward_sign();
    test_stop_is_immediate();
    test_no_windup();
    test_blocked_wheel_bounded();
    test_reversal();
    test_elapsed_ticks();
    test_counts();
    test_twist();
    test_sign_change_clears_integral();
    test_integral_zone();
    test_period_two_rejected();
    test_stall_start_pushes();
    test_brakes_through_dead_zone();
    test_coasts_near_setpoint();
    printf("%d passed, %d failed\n", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
