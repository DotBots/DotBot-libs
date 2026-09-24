/**
 * @file
 * @ingroup drv_steering
 *
 * @brief  Steering to a single target
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "geometry.h"
#include "steering.h"
#include "wheel_control.h"

//=========================== defines ==========================================

#define DEG_TO_RAD ((float)M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / (float)M_PI)

//=========================== private ==========================================

static float _wrap180(float deg) {
    while (deg >= 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

/// The smaller rotation that puts a bearing on the heading line, either end
static float _wrap90(float deg) {
    deg = _wrap180(deg);
    if (deg > 90.0f) {
        return deg - 180.0f;
    }
    if (deg < -90.0f) {
        return deg + 180.0f;
    }
    return deg;
}

static float _clamp(float value, float low, float high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static float _sign(float value) {
    return (value < 0) ? -1.0f : 1.0f;
}

/// Body-forward unit vector for a heading
static void _forward(float heading_deg, float *fx, float *fy) {
    *fx = -sinf(heading_deg * DEG_TO_RAD);
    *fy = cosf(heading_deg * DEG_TO_RAD);
}

static void _enter(db_steering_t *steering, db_steering_state_t state) {
    if (state != steering->state) {
        steering->spinning = false;
    }
    steering->state       = state;
    steering->state_ticks = 0;
}

static void _fail(db_steering_t *steering, db_steering_fail_t reason) {
    _enter(steering, DB_STEERING_FAILED);
    steering->fail = reason;
}

static void _reset_progress(db_steering_t *steering) {
    steering->best_mm        = INFINITY;
    steering->progress_ticks = 0;
}

/// Turn rate from the heading error, deg/s
static float _heading_pd(db_steering_t *steering, float error_deg, uint32_t elapsed_ticks) {
    const db_steering_conf_t *conf  = steering->conf;
    float                     omega = conf->heading_kp * error_deg;
    if (steering->has_error && elapsed_ticks > 0) {
        float dt = (float)(elapsed_ticks * DB_STEERING_TICK_MS) / 1000.0f;
        omega += conf->heading_kd * _wrap180(error_deg - steering->error_deg) / dt;
    }
    steering->error_deg = error_deg;
    steering->has_error = true;
    return omega;
}

/// Mix, cap the turn at the spin limit, and keep what was commanded for the prediction
static void _command(db_steering_t *steering, float v_mm_s, float omega_deg_s, bool in_place, float error_deg, db_steering_output_t *out) {
    const db_steering_conf_t *conf  = steering->conf;
    db_body_twist_t           twist = { .v_mm_s = v_mm_s, .omega_deg_s = omega_deg_s };
    float                     left;
    float                     right;
    db_wheel_control_from_twist(&twist, &left, &right);

    float mean = (left + right) / 2.0f;
    float diff = _clamp((left - right) / 2.0f, -conf->spin_mm_s, conf->spin_mm_s);
    if (in_place && fabsf(diff) < conf->spin_min_mm_s) {
        diff = _sign(error_deg) * conf->spin_min_mm_s;
    }
    out->left_mm_s  = mean + diff;
    out->right_mm_s = mean - diff;
    out->brake      = false;

    steering->v_mm_s      = mean;
    steering->omega_deg_s = 2.0f * diff / db_track_effective_mm(out->left_mm_s, out->right_mm_s) * RAD_TO_DEG;
}

static void _halt(db_steering_t *steering, bool brake, db_steering_output_t *out) {
    out->left_mm_s        = 0;
    out->right_mm_s       = 0;
    out->brake            = brake;
    steering->v_mm_s      = 0;
    steering->omega_deg_s = 0;
    steering->has_error   = false;
}

/// ALIGN, DRIVE and FINAL_TURN, on a tracking pose
static void _move(db_steering_t *steering, const db_steering_pose_t *pose, uint32_t elapsed_ticks, db_steering_output_t *out) {
    const db_steering_conf_t   *conf   = steering->conf;
    const db_steering_target_t *target = &steering->target;

    // The steered point and its goal: the photodiode onto the target, or with a
    // final heading the axle onto the point a lever arm behind it
    float lever  = conf->lever_mm;
    float goal_x = target->x_mm;
    float goal_y = target->y_mm;
    if (target->has_final_heading) {
        float gx, gy;
        _forward(target->final_heading_deg, &gx, &gy);
        goal_x -= lever * gx;
        goal_y -= lever * gy;
        lever = 0;
    }

    float fx, fy;
    _forward(pose->heading_deg, &fx, &fy);

    if (steering->state == DB_STEERING_FINAL_TURN) {
        float heading = pose->heading_deg + steering->omega_deg_s * conf->lookahead_s;
        float error   = _wrap180(target->final_heading_deg - heading);
        if (fabsf(_wrap180(target->final_heading_deg - pose->heading_deg)) < conf->final_tol_deg && fabsf(error) < conf->final_tol_deg) {
            _enter(steering, DB_STEERING_ARRIVED);
            _halt(steering, true, out);
            return;
        }
        if (steering->state_ticks > conf->turn_ticks) {
            _fail(steering, DB_STEERING_FAIL_TURN);
            _halt(steering, true, out);
            return;
        }
        _command(steering, 0, _heading_pd(steering, error, elapsed_ticks), true, error, out);
        return;
    }

    // Arrival: where the steered point stops if braked now
    float point_x   = pose->x_mm + lever * fx;
    float point_y   = pose->y_mm + lever * fy;
    float runon     = steering->v_mm_s * conf->runon_s;
    float threshold = fmaxf(target->threshold_mm, conf->arrival_min_mm);
    float distance  = hypotf(goal_x - point_x, goal_y - point_y);
    if (hypotf(goal_x - point_x - runon * fx, goal_y - point_y - runon * fy) <= threshold) {
        if (target->has_final_heading) {
            _enter(steering, DB_STEERING_FINAL_TURN);
            steering->has_error = false;
            _move(steering, pose, elapsed_ticks, out);
            return;
        }
        _enter(steering, DB_STEERING_ARRIVED);
        _halt(steering, true, out);
        return;
    }

    steering->distance_mm = distance;
    if (distance < steering->best_mm - conf->progress_mm) {
        steering->best_mm        = distance;
        steering->progress_ticks = 0;
    } else {
        steering->progress_ticks += elapsed_ticks;
        if (steering->progress_ticks > conf->progress_ticks) {
            _fail(steering, DB_STEERING_FAIL_PROGRESS);
            _halt(steering, true, out);
            return;
        }
    }

    // Errors from the pose one lookahead ahead along the last command
    float turn    = steering->omega_deg_s * conf->lookahead_s;
    float heading = pose->heading_deg + turn;
    float mx, my;
    _forward(pose->heading_deg + turn / 2.0f, &mx, &my);
    float ax = pose->x_mm + steering->v_mm_s * conf->lookahead_s * mx;
    float ay = pose->y_mm + steering->v_mm_s * conf->lookahead_s * my;
    _forward(heading, &fx, &fy);

    float rx    = goal_x - ax;
    float ry    = goal_y - ay;
    float reach = hypotf(rx, ry);
    float error = 0;
    if (reach >= conf->bearing_min_mm) {
        error = _wrap180(atan2f(-rx, ry) * RAD_TO_DEG - heading);
        if (reach < lever + conf->near_mm) {
            error = _wrap90(error);
        }
    }
    float along = rx * fx + ry * fy - lever;

    if (steering->state == DB_STEERING_DRIVE && fabsf(error) > conf->align_enter_deg) {
        _enter(steering, DB_STEERING_ALIGN);
    } else if (steering->state == DB_STEERING_ALIGN && fabsf(error) < conf->align_exit_deg) {
        _enter(steering, DB_STEERING_DRIVE);
    }

    float omega = _heading_pd(steering, error, elapsed_ticks);
    if (steering->state == DB_STEERING_ALIGN) {
        if (steering->state_ticks > conf->turn_ticks) {
            _fail(steering, DB_STEERING_FAIL_TURN);
            _halt(steering, true, out);
            return;
        }
        _command(steering, 0, omega, true, error, out);
        return;
    }

    float speed = fminf(conf->v_max_mm_s, conf->approach_per_s * fabsf(along));
    float scale = _clamp((conf->align_enter_deg - fabsf(error)) / (conf->align_enter_deg - conf->full_speed_deg), 0, 1);
    _command(steering, _sign(along) * speed * scale, omega, false, error, out);
}

//=========================== public ===========================================

void db_steering_init(db_steering_t *steering, const db_steering_conf_t *conf) {
    steering->conf        = conf;
    steering->fail        = DB_STEERING_FAIL_NONE;
    steering->v_mm_s      = 0;
    steering->omega_deg_s = 0;
    steering->has_error   = false;
    steering->distance_mm = 0;
    steering->spinning    = false;
    steering->state       = DB_STEERING_IDLE;
    _reset_progress(steering);
    _enter(steering, DB_STEERING_IDLE);
}

void db_steering_set_target(db_steering_t *steering, const db_steering_target_t *target) {
    steering->target    = *target;
    steering->fail      = DB_STEERING_FAIL_NONE;
    steering->has_error = false;
    _reset_progress(steering);
    switch (steering->state) {
        case DB_STEERING_DRIVE:
        case DB_STEERING_NO_HEADING:
            _enter(steering, steering->state);
            break;
        case DB_STEERING_ALIGN:
        case DB_STEERING_FINAL_TURN:
            _enter(steering, DB_STEERING_ALIGN);
            break;
        default:
            // From rest: NO_HEADING goes on to ALIGN at once if the pose tracks
            _enter(steering, DB_STEERING_NO_HEADING);
            break;
    }
}

void db_steering_stop(db_steering_t *steering) {
    steering->v_mm_s      = 0;
    steering->omega_deg_s = 0;
    steering->has_error   = false;
    steering->fail        = DB_STEERING_FAIL_NONE;
    _enter(steering, DB_STEERING_IDLE);
}

bool db_steering_active(const db_steering_t *steering) {
    return steering->state != DB_STEERING_IDLE && steering->state != DB_STEERING_ARRIVED && steering->state != DB_STEERING_FAILED;
}

void db_steering_step(db_steering_t *steering, const db_steering_pose_t *pose, uint32_t elapsed_ticks, db_steering_output_t *out) {
    const db_steering_conf_t *conf = steering->conf;
    steering->state_ticks += elapsed_ticks;

    switch (steering->state) {
        case DB_STEERING_IDLE:
            _halt(steering, false, out);
            return;
        case DB_STEERING_ARRIVED:
        case DB_STEERING_FAILED:
            _halt(steering, true, out);
            return;
        case DB_STEERING_HOLD:
            if (pose->status == DB_STEERING_POSE_SEEDING) {
                _fail(steering, DB_STEERING_FAIL_HEADING_LOST);
            } else if (pose->status == DB_STEERING_POSE_TRACKING) {
                _enter(steering, DB_STEERING_ALIGN);
                _reset_progress(steering);
                _move(steering, pose, elapsed_ticks, out);
                return;
            } else if (steering->state_ticks > conf->hold_ticks) {
                _fail(steering, DB_STEERING_FAIL_HOLD);
            }
            _halt(steering, true, out);
            return;
        case DB_STEERING_NO_HEADING:
            if (pose->status == DB_STEERING_POSE_TRACKING && (!steering->spinning || steering->state_ticks >= conf->no_heading_turn_ticks)) {
                steering->spinning  = false;
                steering->has_error = false;
                _enter(steering, DB_STEERING_ALIGN);
                _move(steering, pose, elapsed_ticks, out);
                return;
            }
            if (pose->status == DB_STEERING_POSE_LOST) {
                _enter(steering, DB_STEERING_HOLD);
                _halt(steering, true, out);
                return;
            }
            if (steering->state_ticks > conf->no_heading_ticks) {
                _fail(steering, DB_STEERING_FAIL_NO_HEADING);
                _halt(steering, true, out);
                return;
            }
            steering->spinning    = true;
            out->left_mm_s        = conf->spin_mm_s;
            out->right_mm_s       = -conf->spin_mm_s;
            out->brake            = false;
            steering->v_mm_s      = 0;
            steering->omega_deg_s = 2.0f * conf->spin_mm_s / db_track_effective_mm(conf->spin_mm_s, -conf->spin_mm_s) * RAD_TO_DEG;
            return;
        default:
            break;
    }

    // ALIGN, DRIVE, FINAL_TURN
    if (pose->status == DB_STEERING_POSE_SEEDING) {
        _fail(steering, DB_STEERING_FAIL_HEADING_LOST);
        _halt(steering, true, out);
        return;
    }
    if (pose->status == DB_STEERING_POSE_LOST) {
        _enter(steering, DB_STEERING_HOLD);
        _halt(steering, true, out);
        return;
    }
    _move(steering, pose, elapsed_ticks, out);
}
