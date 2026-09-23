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

/// Consecutive steps without a count before a turning wheel counts as stalled
/// again. Below about 25 mm/s a rolling wheel also goes this long between
/// counts, so the kick returns there; that is the price of detecting a stall.
#define STALL_TICKS (4U)

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
    wheel->still_ticks = STALL_TICKS;
    wheel->kick_boost  = 0;
}

void db_wheel_control_set_setpoint(db_wheel_control_t *wheel, float mm_per_s) {
    if (mm_per_s * wheel->setpoint <= 0) {
        wheel->integral = 0;
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
    } else if (wheel->still_ticks < STALL_TICKS) {
        wheel->still_ticks++;
    }

    if (wheel->setpoint == 0) {
        wheel->kick_boost = 0;
        wheel->integral   = 0;
        wheel->pwm        = 0;
        wheel->ff         = 0;
        return 0;
    }

    // The error takes the mean of the last two speeds: a wheel that answers
    // within one step otherwise sustains a cycle alternating every step, and
    // a two-step mean has no gain at that frequency
    float speed = 0.5f * (wheel->measured + wheel->previous);
    float sign  = (wheel->setpoint > 0) ? 1.0f : -1.0f;
    float error = wheel->setpoint - speed;

    // A stalled wheel gets the kick, and the P term on the whole setpoint so a
    // step from rest starts with the push the running wheel will need; the
    // integral stays out of it
    float run = fminf(conf->u_run + conf->k_run * fabsf(wheel->setpoint), conf->pwm_max);
    if (wheel->still_ticks >= STALL_TICKS) {
        float kick        = fmaxf(conf->u_breakaway, run);
        wheel->kick_boost = fminf(wheel->kick_boost, conf->pwm_max - kick);
        wheel->ff         = sign * (kick + wheel->kick_boost);
        wheel->kick_boost += conf->kick_ramp;
        error = wheel->setpoint;
    } else {
        wheel->kick_boost = 0;
        wheel->ff         = sign * run;
        // Outside the zone the wheel is still getting up to speed, and what
        // the integral would store there is the overshoot at the end of it
        if (fabsf(error) < conf->i_zone) {
            wheel->integral += error * dt;
        }
    }

    // Anti-windup: the integral may only hold what the output range still has
    // room for once the feedforward is applied
    if (conf->ki > 0) {
        wheel->integral = _clamp(wheel->integral, (-conf->pwm_max - wheel->ff) / conf->ki, (conf->pwm_max - wheel->ff) / conf->ki);
    } else {
        wheel->integral = 0;
    }

    float pwm  = wheel->ff + conf->kp * error + conf->ki * wheel->integral;
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
    *left_mm_s  = twist->v_mm_s + w * DB_TRACK / 2.0f;
    *right_mm_s = twist->v_mm_s - w * DB_TRACK / 2.0f;
}
