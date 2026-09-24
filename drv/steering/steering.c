/**
 * @file
 * @ingroup drv_steering
 *
 * @brief  Steering along waypoints
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <string.h>

#include "geometry.h"
#include "protocol.h"
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
    steering->fail       = reason;
    steering->completion = DB_STEERING_DONE_FAILED;
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

/// Whether the recovery straight from the last tracked pose stays inside the bounds
static bool _recover_clear(const db_steering_t *steering, float sign) {
    const db_steering_conf_t *conf = steering->conf;
    const float              *b    = conf->bounds_mm;
    if (b[0] == 0 && b[1] == 0 && b[2] == 0 && b[3] == 0) {
        return true;
    }
    float fx, fy;
    _forward(steering->last_heading_deg, &fx, &fy);
    float reach = sign * (conf->recover_mm + conf->recover_mm_s * conf->runon_s);
    float x     = steering->last_x_mm + reach * fx;
    float y     = steering->last_y_mm + reach * fy;
    float m     = conf->bounds_margin_mm;
    return x >= b[0] + m && x <= b[2] - m && y >= b[1] + m && y <= b[3] - m;
}

/// The heading was lost: straight on if moving and allowed, else spin as from rest
static void _heading_lost(db_steering_t *steering, db_steering_output_t *out) {
    const db_steering_conf_t *conf   = steering->conf;
    bool                      moving = fabsf(steering->v_mm_s) > 1.0f;
    float                     sign   = (steering->v_mm_s < 0) ? -1.0f : 1.0f;
    if (conf->recover == DB_STEERING_RECOVER_DRIVE && moving && steering->has_last && _recover_clear(steering, sign)) {
        _enter(steering, DB_STEERING_RECOVER);
        steering->recover_sign = sign;
        steering->has_error    = false;
        out->left_mm_s         = sign * conf->recover_mm_s;
        out->right_mm_s        = sign * conf->recover_mm_s;
        out->brake             = false;
        steering->v_mm_s       = sign * conf->recover_mm_s;
        steering->omega_deg_s  = 0;
        return;
    }
    _enter(steering, DB_STEERING_NO_HEADING);
    _halt(steering, false, out);
}

/// Latch the arrival at the last point
static void _arrive(db_steering_t *steering, db_steering_output_t *out) {
    _enter(steering, DB_STEERING_ARRIVED);
    steering->index      = steering->path.count;
    steering->completion = DB_STEERING_DONE_ARRIVED;
    _halt(steering, true, out);
}

/// The current point as a target, for the app and the debugger
static void _load_target(db_steering_t *steering) {
    const db_steering_point_t *point = &steering->path.points[steering->index];
    steering->target                 = (db_steering_target_t){
                        .x_mm              = point->x_mm,
                        .y_mm              = point->y_mm,
                        .threshold_mm      = steering->path.threshold_mm,
                        .has_final_heading = point->has_heading,
                        .final_heading_deg = point->heading_deg,
    };
}

/// On to the next point, keeping the move going
static void _advance(db_steering_t *steering) {
    steering->index++;
    _load_target(steering);
    _reset_progress(steering);
    steering->has_error = false;
}

static bool _is_last(const db_steering_t *steering) {
    return steering->index + 1U >= steering->path.count;
}

/// A threshold below the least reached by driving asks for the precise arrival
static bool _is_precise(const db_steering_t *steering) {
    return _is_last(steering) && !steering->path.points[steering->index].has_heading && steering->path.threshold_mm < steering->conf->arrival_min_mm;
}

static float _precise_threshold(const db_steering_t *steering) {
    return fmaxf(steering->path.threshold_mm, steering->conf->precise_min_mm);
}

/// Start of the current leg: the previous point, or where the batch started
static void _leg_start(const db_steering_t *steering, float *x, float *y) {
    if (steering->index > 0) {
        *x = steering->path.points[steering->index - 1].x_mm;
        *y = steering->path.points[steering->index - 1].y_mm;
    } else {
        *x = steering->start_x_mm;
        *y = steering->start_y_mm;
    }
}

/// Speed the approach to an intermediate point keeps for the turn onto the next leg
static float _exit_speed(const db_steering_t *steering) {
    const db_steering_conf_t  *conf = steering->conf;
    const db_steering_point_t *here = &steering->path.points[steering->index];
    const db_steering_point_t *next = &steering->path.points[steering->index + 1];
    float                      sx, sy;
    _leg_start(steering, &sx, &sy);
    float in_x = here->x_mm - sx, in_y = here->y_mm - sy;
    float out_x = next->x_mm - here->x_mm, out_y = next->y_mm - here->y_mm;
    float in_len = hypotf(in_x, in_y), out_len = hypotf(out_x, out_y);
    if (next->has_heading || in_len < 1.0f || out_len < 1.0f) {
        return 0;
    }
    float turn = acosf(_clamp((in_x * out_x + in_y * out_y) / (in_len * out_len), -1.0f, 1.0f)) * RAD_TO_DEG;
    return steering->v_max_mm_s * _clamp((conf->align_enter_deg - turn) / (conf->align_enter_deg - conf->full_speed_deg), 0, 1);
}

/// Whether the axle has passed an intermediate point: within the pass radius,
/// or beyond it along the leg, so a point it cannot quite reach is not orbited
static bool _passed(const db_steering_t *steering, float px, float py) {
    const db_steering_point_t *here = &steering->path.points[steering->index];
    float                      pass = (steering->path.pass_mm > 0) ? steering->path.pass_mm : steering->conf->pass_mm;
    if (hypotf(here->x_mm - px, here->y_mm - py) <= pass) {
        return true;
    }
    float sx, sy;
    _leg_start(steering, &sx, &sy);
    float ux = here->x_mm - sx, uy = here->y_mm - sy;
    return hypotf(ux, uy) >= 1.0f && (px - here->x_mm) * ux + (py - here->y_mm) * uy >= 0;
}

/// Whether the creep into a precise point should stop: the stop point is inside
/// half the threshold, or at its closest to the point
static bool _precise_reached(const db_steering_t *steering, const db_steering_pose_t *pose) {
    const db_steering_conf_t *conf = steering->conf;
    float                     fx, fy;
    _forward(pose->heading_deg, &fx, &fy);
    float runon = steering->v_mm_s * conf->runon_s;
    float ex    = steering->target.x_mm - (pose->x_mm + runon * fx);
    float ey    = steering->target.y_mm - (pose->y_mm + runon * fy);
    float dist  = hypotf(ex, ey);
    if (dist <= 0.5f * _precise_threshold(steering)) {
        return true;
    }
    float ahead = (ex * fx + ey * fy) * _sign(steering->v_mm_s);
    return fabsf(steering->v_mm_s) > 1.0f && dist < conf->arrival_min_mm && ahead <= 0;
}

static void _settle(db_steering_t *steering, db_steering_output_t *out) {
    _enter(steering, DB_STEERING_SETTLE);
    steering->settle_x_mm  = 0;
    steering->settle_y_mm  = 0;
    steering->settle_count = 0;
    _halt(steering, true, out);
}

/// Wheel speeds of the correction in progress
static void _nudge_command(db_steering_t *steering, db_steering_output_t *out) {
    const db_steering_conf_t *conf = steering->conf;
    float                     sign = _sign(steering->nudge_goal);
    if (steering->nudge_turn) {
        out->left_mm_s  = sign * conf->spin_min_mm_s;
        out->right_mm_s = -sign * conf->spin_min_mm_s;
    } else {
        out->left_mm_s  = sign * conf->creep_mm_s;
        out->right_mm_s = sign * conf->creep_mm_s;
    }
    out->brake       = false;
    steering->v_mm_s = steering->nudge_turn ? 0 : sign * conf->creep_mm_s;
}

/// The fixes at rest are in: arrived, or correct what is left
static void _settle_check(db_steering_t *steering, const db_steering_pose_t *pose, db_steering_output_t *out) {
    const db_steering_conf_t *conf = steering->conf;
    float                     fx, fy;
    _forward(pose->heading_deg, &fx, &fy);
    // The axle at rest: the mean fix, a lever arm back along the heading
    float mx  = steering->settle_x_mm / (float)steering->settle_count - conf->lever_mm * fx;
    float my  = steering->settle_y_mm / (float)steering->settle_count - conf->lever_mm * fy;
    float ex  = steering->target.x_mm - mx;
    float ey  = steering->target.y_mm - my;
    float thr = _precise_threshold(steering);
    if (hypotf(ex, ey) <= thr) {
        _arrive(steering, out);
        return;
    }
    if (steering->nudges >= conf->settle_nudges) {
        _fail(steering, DB_STEERING_FAIL_SETTLE);
        _halt(steering, true, out);
        return;
    }
    steering->nudges++;
    // Off the heading line, turn in place until the line points at the goal,
    // either end; on it, creep along it
    float along                 = ex * fx + ey * fy;
    float lateral               = ex * fy - ey * fx;
    steering->nudge_turn        = fabsf(lateral) > 0.5f * thr;
    steering->nudge_goal        = steering->nudge_turn ? _wrap90(atan2f(-ex, ey) * RAD_TO_DEG - pose->heading_deg) : along;
    steering->nudge_done        = 0;
    steering->nudge_rate        = 0;
    steering->nudge_x_mm        = pose->x_mm;
    steering->nudge_y_mm        = pose->y_mm;
    steering->nudge_heading_deg = pose->heading_deg;
    _enter(steering, DB_STEERING_NUDGE);
    _nudge_command(steering, out);
}

/// ALIGN, DRIVE and FINAL_TURN, on a tracking pose
static void _move(db_steering_t *steering, const db_steering_pose_t *pose, uint32_t elapsed_ticks, db_steering_output_t *out) {
    const db_steering_conf_t  *conf  = steering->conf;
    const db_steering_point_t *point = &steering->path.points[steering->index];

    float hx, hy;
    _forward(pose->heading_deg, &hx, &hy);
    steering->last_x_mm        = pose->x_mm + conf->lever_mm * hx;
    steering->last_y_mm        = pose->y_mm + conf->lever_mm * hy;
    steering->last_heading_deg = pose->heading_deg;
    steering->has_last         = true;
    if (!steering->has_start) {
        steering->start_x_mm = pose->x_mm;
        steering->start_y_mm = pose->y_mm;
        steering->has_start  = true;
    }

    bool last    = _is_last(steering);
    bool precise = _is_precise(steering);

    // An intermediate point is passed, not stopped at
    if (!last && !point->has_heading && _passed(steering, pose->x_mm, pose->y_mm)) {
        _advance(steering);
        _move(steering, pose, elapsed_ticks, out);
        return;
    }

    float goal_x = point->x_mm;
    float goal_y = point->y_mm;

    float fx, fy;
    _forward(pose->heading_deg, &fx, &fy);

    if (steering->state == DB_STEERING_FINAL_TURN) {
        float tol     = (steering->path.heading_tol_deg > 0) ? steering->path.heading_tol_deg : conf->final_tol_deg;
        float heading = pose->heading_deg + steering->omega_deg_s * conf->lookahead_s;
        float error   = _wrap180(point->heading_deg - heading);
        if (fabsf(_wrap180(point->heading_deg - pose->heading_deg)) < tol && fabsf(error) < tol) {
            if (last) {
                _arrive(steering, out);
                return;
            }
            _advance(steering);
            _enter(steering, DB_STEERING_ALIGN);
            _move(steering, pose, elapsed_ticks, out);
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

    // Arrival: where the axle stops if braked now. A precise one creeps in
    // and stops on the poll, then settles.
    float point_x   = pose->x_mm;
    float point_y   = pose->y_mm;
    float runon     = steering->v_mm_s * conf->runon_s;
    float threshold = last || point->has_heading ? fmaxf(steering->path.threshold_mm, conf->arrival_min_mm) : ((steering->path.pass_mm > 0) ? steering->path.pass_mm : conf->pass_mm);
    float distance  = hypotf(goal_x - point_x, goal_y - point_y);
    if (precise) {
        if (steering->state == DB_STEERING_DRIVE && _precise_reached(steering, pose)) {
            _settle(steering, out);
            return;
        }
    } else if ((last || point->has_heading) && hypotf(goal_x - point_x - runon * fx, goal_y - point_y - runon * fy) <= threshold) {
        if (point->has_heading) {
            _enter(steering, DB_STEERING_FINAL_TURN);
            steering->has_error = false;
            _move(steering, pose, elapsed_ticks, out);
            return;
        }
        _arrive(steering, out);
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
        if (reach < conf->near_mm) {
            // Near the axle, turn only as far as it takes to bring the heading
            // line within half the threshold of the target, then drive along
            // the line: the full bearing swings with every millimetre there and
            // flips between the two ends of the line
            float target_cross = 0.5f * threshold;
            float line         = _wrap90(error);
            float enough       = (reach > target_cross) ? asinf(target_cross / reach) * RAD_TO_DEG : 90.0f;
            error              = (fabsf(line) > enough) ? line - _sign(line) * enough : 0;
        }
    }
    float along = rx * fx + ry * fy;

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

    float speed = conf->approach_per_s * fabsf(along);
    if (!last && !point->has_heading) {
        speed += _exit_speed(steering);
    }
    speed = fminf(steering->v_max_mm_s, speed);
    if (precise) {
        speed = fmaxf(speed, conf->creep_mm_s);
    }
    float scale = _clamp((conf->align_enter_deg - fabsf(error)) / (conf->align_enter_deg - conf->full_speed_deg), 0, 1);
    _command(steering, _sign(along) * speed * scale, omega, false, error, out);
}

//=========================== public ===========================================

void db_steering_init(db_steering_t *steering, const db_steering_conf_t *conf) {
    steering->conf         = conf;
    steering->fail         = DB_STEERING_FAIL_NONE;
    steering->completion   = DB_STEERING_DONE_NONE;
    steering->v_max_mm_s   = conf->v_max_mm_s;
    steering->v_mm_s       = 0;
    steering->omega_deg_s  = 0;
    steering->has_error    = false;
    steering->distance_mm  = 0;
    steering->spinning     = false;
    steering->has_last     = false;
    steering->recover_sign = 1.0f;
    steering->index        = 0;
    steering->path.count   = 0;
    steering->state        = DB_STEERING_IDLE;
    _reset_progress(steering);
    _enter(steering, DB_STEERING_IDLE);
}

void db_steering_set_target(db_steering_t *steering, const db_steering_target_t *target) {
    db_steering_path_t path = {
        .points       = { { .x_mm = target->x_mm, .y_mm = target->y_mm, .has_heading = target->has_final_heading, .heading_deg = target->final_heading_deg } },
        .count        = 1,
        .threshold_mm = target->threshold_mm,
    };
    db_steering_set_path(steering, &path);
}

void db_steering_set_path(db_steering_t *steering, const db_steering_path_t *path) {
    steering->path = *path;
    if (steering->path.count > DB_STEERING_MAX_POINTS) {
        steering->path.count = DB_STEERING_MAX_POINTS;
    }
    steering->index      = 0;
    steering->has_start  = false;
    steering->nudges     = 0;
    steering->fail       = DB_STEERING_FAIL_NONE;
    steering->completion = DB_STEERING_DONE_IN_PROGRESS;
    steering->has_error  = false;
    _load_target(steering);
    _reset_progress(steering);
    switch (steering->state) {
        case DB_STEERING_DRIVE:
        case DB_STEERING_NO_HEADING:
            _enter(steering, steering->state);
            break;
        case DB_STEERING_ALIGN:
        case DB_STEERING_FINAL_TURN:
        case DB_STEERING_SETTLE:
        case DB_STEERING_NUDGE:
            _enter(steering, DB_STEERING_ALIGN);
            break;
        default:
            // From rest: NO_HEADING goes on to ALIGN at once if the pose tracks
            _enter(steering, DB_STEERING_NO_HEADING);
            break;
    }
}

bool db_steering_path_from_wire(const uint8_t *payload, size_t length, db_steering_path_t *path, uint8_t *batch_id) {
    *path     = (db_steering_path_t){ 0 };
    *batch_id = 0;
    uint16_t threshold;
    if (length < sizeof(threshold) + 1U) {
        return false;
    }
    memcpy(&threshold, payload, sizeof(threshold));
    size_t count  = payload[sizeof(threshold)];
    size_t points = sizeof(threshold) + 1U;
    if (length < points + count * sizeof(protocol_lh2_location_t)) {
        return false;
    }
    size_t trailer  = points + count * sizeof(protocol_lh2_location_t);
    size_t headings = trailer + sizeof(protocol_lh2_waypoints_trailer_t);
    bool   full     = length >= headings + count * sizeof(int16_t);
    size_t kept     = (count > DB_STEERING_MAX_POINTS) ? DB_STEERING_MAX_POINTS : count;

    path->count        = (uint8_t)kept;
    path->threshold_mm = (float)threshold;
    if (full) {
        protocol_lh2_waypoints_trailer_t t;
        memcpy(&t, &payload[trailer], sizeof(t));
        *batch_id             = t.batch_id;
        path->heading_tol_deg = (float)t.heading_tol_deg;
        path->pass_mm         = (float)t.pass_mm;
    }
    for (size_t i = 0; i < kept; i++) {
        protocol_lh2_location_t point;
        memcpy(&point, &payload[points + i * sizeof(point)], sizeof(point));
        path->points[i].x_mm = (float)point.x;
        path->points[i].y_mm = (float)point.y;
        if (full) {
            int16_t heading;
            memcpy(&heading, &payload[headings + i * sizeof(heading)], sizeof(heading));
            path->points[i].has_heading = heading != DB_WAYPOINT_NO_HEADING;
            path->points[i].heading_deg = (float)heading / 100.0f;
        }
    }
    return true;
}

void db_steering_set_max_speed(db_steering_t *steering, float v_mm_s) {
    steering->v_max_mm_s = (v_mm_s > 0) ? v_mm_s : steering->conf->v_max_mm_s;
}

void db_steering_stop(db_steering_t *steering) {
    if (db_steering_active(steering)) {
        steering->completion = DB_STEERING_DONE_ABORTED;
    }
    steering->v_mm_s      = 0;
    steering->omega_deg_s = 0;
    steering->has_error   = false;
    steering->fail        = DB_STEERING_FAIL_NONE;
    _enter(steering, DB_STEERING_IDLE);
}

bool db_steering_active(const db_steering_t *steering) {
    return steering->state != DB_STEERING_IDLE && steering->state != DB_STEERING_ARRIVED && steering->state != DB_STEERING_FAILED;
}

void db_steering_fix(db_steering_t *steering, float x_mm, float y_mm) {
    if (steering->state != DB_STEERING_SETTLE || steering->state_ticks < steering->conf->settle_skip_ticks) {
        return;
    }
    steering->settle_x_mm += x_mm;
    steering->settle_y_mm += y_mm;
    steering->settle_count++;
}

bool db_steering_poll(db_steering_t *steering, const db_steering_pose_t *pose, db_steering_output_t *out) {
    if (pose->status != DB_STEERING_POSE_TRACKING) {
        return false;
    }
    if (steering->state == DB_STEERING_DRIVE && _is_precise(steering) && _precise_reached(steering, pose)) {
        _settle(steering, out);
        return true;
    }
    if (steering->state != DB_STEERING_NUDGE) {
        return false;
    }
    // Progress of the correction, and where it would end if braked now
    float done;
    if (steering->nudge_turn) {
        done = _wrap180(pose->heading_deg - steering->nudge_heading_deg);
    } else {
        float fx, fy;
        _forward(steering->nudge_heading_deg, &fx, &fy);
        done = (pose->x_mm - steering->nudge_x_mm) * fx + (pose->y_mm - steering->nudge_y_mm) * fy;
    }
    steering->nudge_rate = (done - steering->nudge_done) * 1000.0f / (float)DB_STEERING_TICK_MS;
    steering->nudge_done = done;
    float sign           = _sign(steering->nudge_goal);
    if (sign * (done + steering->nudge_rate * steering->conf->runon_s) >= fabsf(steering->nudge_goal)) {
        _settle(steering, out);
        return true;
    }
    return false;
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
                // Standing still: re-acquire by the spin
                _enter(steering, DB_STEERING_NO_HEADING);
                _halt(steering, false, out);
                return;
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
        case DB_STEERING_RECOVER:
            if (pose->status == DB_STEERING_POSE_TRACKING) {
                _enter(steering, DB_STEERING_ALIGN);
                _reset_progress(steering);
                steering->has_error = false;
                _move(steering, pose, elapsed_ticks, out);
                return;
            }
            if (pose->status == DB_STEERING_POSE_LOST) {
                _enter(steering, DB_STEERING_HOLD);
                _halt(steering, true, out);
                return;
            }
            if ((float)(steering->state_ticks * DB_STEERING_TICK_MS) / 1000.0f * conf->recover_mm_s >= conf->recover_mm) {
                _fail(steering, DB_STEERING_FAIL_HEADING_LOST);
                _halt(steering, true, out);
                return;
            }
            out->left_mm_s  = steering->recover_sign * conf->recover_mm_s;
            out->right_mm_s = steering->recover_sign * conf->recover_mm_s;
            out->brake      = false;
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

    // ALIGN, DRIVE, FINAL_TURN, SETTLE, NUDGE
    if (pose->status == DB_STEERING_POSE_SEEDING) {
        _heading_lost(steering, out);
        return;
    }
    if (pose->status == DB_STEERING_POSE_LOST) {
        _enter(steering, DB_STEERING_HOLD);
        _halt(steering, true, out);
        return;
    }
    if (steering->state == DB_STEERING_SETTLE) {
        if (steering->settle_count >= conf->settle_fixes) {
            _settle_check(steering, pose, out);
            return;
        }
        if (steering->state_ticks > conf->settle_ticks) {
            _fail(steering, DB_STEERING_FAIL_SETTLE);
        }
        _halt(steering, true, out);
        return;
    }
    if (steering->state == DB_STEERING_NUDGE) {
        if (steering->state_ticks > conf->nudge_ticks) {
            _settle(steering, out);
            return;
        }
        _nudge_command(steering, out);
        return;
    }
    _move(steering, pose, elapsed_ticks, out);
}
