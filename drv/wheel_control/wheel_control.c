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
    float track = DB_TRACK_EFFECTIVE;
    // The track depends on the radius the wheel speeds give, which depends on
    // the track; the fixed point is a contraction and settles in a few passes
    for (int i = 0; i < 3; i++) {
        track = db_track_effective_mm(twist->v_mm_s + w * track / 2.0f, twist->v_mm_s - w * track / 2.0f);
    }
    *left_mm_s  = twist->v_mm_s + w * track / 2.0f;
    *right_mm_s = twist->v_mm_s - w * track / 2.0f;
}

void db_wheel_goal_start(db_wheel_goal_t *goal, float left_mm, float right_mm, float speed_mm_s) {
    goal->target_left_mm     = left_mm;
    goal->target_right_mm    = right_mm;
    goal->travelled_left_mm  = 0;
    goal->travelled_right_mm = 0;
    goal->speed_mm_s         = speed_mm_s;
    goal->active             = (left_mm != 0 || right_mm != 0) && speed_mm_s > 0;
}

void db_wheel_goal_straight(db_wheel_goal_t *goal, float distance_mm, float speed_mm_s) {
    db_wheel_goal_start(goal, distance_mm, distance_mm, speed_mm_s);
}

void db_wheel_goal_turn(db_wheel_goal_t *goal, float angle_deg, float speed_mm_s) {
    float arc_mm = angle_deg * (float)M_PI / 180.0f * DB_TRACK_EFFECTIVE / 2.0f;
    db_wheel_goal_start(goal, arc_mm, -arc_mm, speed_mm_s);
}

bool db_wheel_goal_step(db_wheel_goal_t *goal, int32_t delta_left, int32_t delta_right, float *left_mm_s, float *right_mm_s) {
    goal->travelled_left_mm += (float)delta_left * DB_MM_PER_COUNT;
    goal->travelled_right_mm += (float)delta_right * DB_MM_PER_COUNT;
    *left_mm_s  = 0;
    *right_mm_s = 0;
    if (!goal->active) {
        return false;
    }

    bool  left_leads = fabsf(goal->target_left_mm) >= fabsf(goal->target_right_mm);
    float target     = left_leads ? goal->target_left_mm : goal->target_right_mm;
    float travelled  = left_leads ? goal->travelled_left_mm : goal->travelled_right_mm;
    // Travel counted in the direction of the target; travel the wrong way is negative
    float progress = (target > 0) ? travelled : -travelled;
    if (progress >= fabsf(target) - DB_WHEEL_GOAL_RUN_ON_S * goal->speed_mm_s) {
        goal->active = false;
        return false;
    }

    float scale = goal->speed_mm_s / fabsf(target);
    *left_mm_s  = goal->target_left_mm * scale;
    *right_mm_s = goal->target_right_mm * scale;
    return true;
}
