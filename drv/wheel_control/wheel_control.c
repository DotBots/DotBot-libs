/**
 * @file
 * @ingroup drv_wheel_control
 *
 * @brief  Per-wheel speed loop
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdint.h>

#include "geometry.h"
#include "wheel_control.h"

//=========================== defines ==========================================

/// Consecutive steps without a count before a wheel counts as standing. A wheel
/// rolling below about 2 mm/s (under one count per 50 ms) reads as standing too.
#define STILL_TICKS (4U)

//=========================== private ==========================================

static float _clamp(float value, float low, float high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

//=========================== public ===========================================

void db_wheel_control_init(db_wheel_control_t *wheel, const db_wheel_control_conf_t *conf) {
    wheel->conf = conf;
    db_wheel_control_reset(wheel);
}

void db_wheel_control_reset(db_wheel_control_t *wheel) {
    wheel->setpoint    = 0;
    wheel->integral    = 0;
    wheel->pwm         = 0;
    wheel->measured    = 0;
    wheel->previous    = 0;
    wheel->ff          = 0;
    wheel->still_ticks = STILL_TICKS;
    wheel->kick_boost  = 0;
    wheel->brake       = false;
    wheel->forced_ms   = 0;
    wheel->stalled     = false;
}

void db_wheel_control_set_setpoint(db_wheel_control_t *wheel, float mm_per_s) {
    if (mm_per_s * wheel->setpoint <= 0) {
        wheel->integral = 0;
    }
    if (mm_per_s != wheel->setpoint) {
        wheel->stalled   = false;
        wheel->forced_ms = 0;
    }
    wheel->setpoint = mm_per_s;
}

int8_t db_wheel_control_step(db_wheel_control_t *wheel, int32_t delta_counts, uint32_t elapsed_ticks) {
    const db_wheel_control_conf_t *conf = wheel->conf;

    if (elapsed_ticks == 0) {
        return (int8_t)wheel->pwm;
    }

    float dt        = (float)elapsed_ticks * (DB_WHEEL_CONTROL_TICK_MS / 1000.0f);
    wheel->previous = wheel->measured;
    wheel->measured = (float)delta_counts * DB_MM_PER_COUNT / dt;

    if (delta_counts != 0) {
        wheel->still_ticks = 0;
    } else if (wheel->still_ticks < STILL_TICKS) {
        wheel->still_ticks++;
    }

    // wheel->pwm is the duty applied over the step just measured
    if (conf->stall_ms > 0 && delta_counts == 0 && fabsf(wheel->pwm) >= conf->stall_pwm) {
        wheel->forced_ms += elapsed_ticks * DB_WHEEL_CONTROL_TICK_MS;
        if (wheel->forced_ms >= conf->stall_ms) {
            wheel->stalled = true;
        }
    } else {
        wheel->forced_ms = 0;
    }

    // A zero setpoint shorts the motor only while the wheel turns; a stalled
    // wheel coasts
    wheel->brake = (wheel->setpoint == 0) && (wheel->still_ticks < STILL_TICKS);
    if (wheel->setpoint == 0 || wheel->stalled) {
        wheel->kick_boost = 0;
        wheel->integral   = 0;
        wheel->pwm        = 0;
        wheel->ff         = 0;
        return 0;
    }

    // Mean of the last two speeds: no gain at the period-two cycle
    float speed = 0.5f * (wheel->measured + wheel->previous);
    float sign  = (wheel->setpoint > 0) ? 1.0f : -1.0f;
    float error = wheel->setpoint - speed;

    // A standing wheel gets the kick and the P term on the whole setpoint, and
    // no integral
    float run = fminf(conf->u_run + conf->k_run * fabsf(wheel->setpoint), conf->pwm_max);
    if (wheel->still_ticks >= STILL_TICKS) {
        float kick        = fmaxf(conf->u_breakaway, run);
        wheel->kick_boost = fminf(wheel->kick_boost, conf->pwm_max - kick);
        wheel->ff         = sign * (kick + wheel->kick_boost);
        wheel->kick_boost += conf->kick_ramp;
        error = wheel->setpoint;
    } else {
        wheel->kick_boost = 0;
        wheel->ff         = sign * run;
        if (fabsf(error) < conf->i_zone) {
            wheel->integral += error * dt;
        }
    }

    // Anti-windup: the integral holds at most the output range the
    // feedforward leaves
    if (conf->ki > 0) {
        wheel->integral = _clamp(wheel->integral, (-conf->pwm_max - wheel->ff) / conf->ki, (conf->pwm_max - wheel->ff) / conf->ki);
    } else {
        wheel->integral = 0;
    }

    float pwm = wheel->ff + conf->kp * error + conf->ki * wheel->integral;
    // Below u_run the motor does not drive: a turning wheel more than i_zone
    // over its setpoint has the command pushed past it, against its motion
    if (wheel->still_ticks < STILL_TICKS && sign * pwm < conf->u_run && sign * error < -conf->i_zone) {
        pwm -= sign * conf->u_run;
    }
    pwm        = _clamp(pwm, -conf->pwm_max, conf->pwm_max);
    pwm        = _clamp(pwm, wheel->pwm - conf->pwm_slew_per_tick, wheel->pwm + conf->pwm_slew_per_tick);
    wheel->pwm = roundf(pwm);
    return (int8_t)wheel->pwm;
}

int32_t db_wheel_control_counts(int32_t acc, uint32_t dbl) {
    if (acc > 0) {
        return acc + 2 * (int32_t)dbl;
    }
    if (acc < 0) {
        return acc - 2 * (int32_t)dbl;
    }
    return 0;
}

void db_wheel_control_from_twist(const db_body_twist_t *twist, float *left_mm_s, float *right_mm_s) {
    float w     = twist->omega_deg_s * (float)M_PI / 180.0f;
    *left_mm_s  = twist->v_mm_s + w * DB_TRACK_EFFECTIVE / 2.0f;
    *right_mm_s = twist->v_mm_s - w * DB_TRACK_EFFECTIVE / 2.0f;
}
