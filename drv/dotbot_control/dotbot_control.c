/**
 * @file
 * @ingroup drv_dotbot_control
 *
 * @brief  The DotBot app's control, hardware-free
 *
 * @copyright Inria, 2026
 */

#include <math.h>
#include <string.h>

#include "dotbot_control.h"
#include "geometry.h"

//=========================== defines ==========================================

#define TICKS_PER_POSITION (10U)  ///< 100 ms, and the rate a new solve is published at
#define TICKS_PER_TIMEOUT  (20U)  ///< 200 ms

/// Largest wheel speed a command or the steering may set, in mm/s. A count
/// then takes 135 us, just over the QDEC's default 128 us sample period.
#define WHEEL_SPEED_MAX_MM_S (700)

/// Cruise speeds a max speed command may set, mm/s
#define MAX_SPEED_MIN_MM_S (20U)
#define MAX_SPEED_MAX_MM_S (700U)

/// Heading uncertainty db_control_seed() starts the estimator with
#define SEED_HEADING_SD_DEG (2.0f)

/// Coordinates above this are the secure side reporting no usable solve.
#define POSITION_INVALID_MM (100000U)

/// Adverts take this share of the node's transmit slots, one per minimum TX
/// interval, and the rest is left for the net core's STATUS frame
#define ADVERT_TX_SHARE_PERCENT (50U)
#define ADVERT_PERIOD_MIN_MS    (100U)   ///< Floor, however short the interval
#define ADVERT_PERIOD_MAX_MS    (1000U)  ///< Ceiling, however long the interval
#define ADVERT_PERIOD_DEF_MS    (500U)   ///< While not joined, when the interval reads 0

_Static_assert(DB_CONTROL_TICK_MS == DB_WHEEL_CONTROL_TICK_MS, "the wheel loop's dt assumes this tick");
_Static_assert(DB_CONTROL_TICK_MS == DB_POSE_ESTIMATOR_TICK_MS, "the estimator's timeout assumes this tick");
_Static_assert(DB_CONTROL_TICK_MS == DB_STEERING_TICK_MS, "the steering's timeouts assume this tick");
_Static_assert(DB_STEERING_PERIOD_TICKS == TICKS_PER_POSITION, "steering runs once per position poll, after it");
_Static_assert((int)DB_STEERING_FAIL_NO_HEADING == (int)DB_WAYPOINTS_FAIL_NO_HEADING && (int)DB_STEERING_FAIL_SETTLE == (int)DB_WAYPOINTS_FAIL_SETTLE,
               "the advertisement carries the steering's fail reason as is");
_Static_assert(DB_STEERING_MAX_POINTS == DB_MAX_WAYPOINTS, "a batch fills the steering");

_Static_assert(sizeof(db_control_input_t) == 24, "db_control_input_t is an ABI");
_Static_assert(sizeof(db_control_output_t) == 8, "db_control_output_t is an ABI");
_Static_assert(sizeof(db_control_report_t) == 64, "db_control_report_t is an ABI");
_Static_assert(offsetof(db_control_report_t, direction) == 44, "db_control_report_t is an ABI");
_Static_assert(DB_CONTROL_ADVERTISEMENT_BYTES == 1U + sizeof(uint16_t) + sizeof(int16_t) + sizeof(protocol_lh2_location_t) + sizeof(uint16_t) + 3U + 2U * sizeof(int32_t) + 2U * sizeof(uint32_t) + 1U + sizeof(protocol_waypoints_report_t),
               "db_control_advertisement() writes exactly this many bytes");
_Static_assert(offsetof(db_control_report_t, pwm_left) == 50, "db_control_report_t is an ABI");

//=========================== variables ========================================

/// Wheel feedforward from an untethered open-loop duty sweep of one v3 on the
/// office carpet (breakaway 37-45, rolling at 32 + 0.092..0.101 duty per mm/s
/// for either wheel and direction); ki from closed-loop holds, kp from step
/// responses. Full duty, and no slew limit short of it.
const db_control_conf_t db_control_default_conf = {
    .wheel = {
        .kp                = 0.52f,
        .ki                = 5.2f,
        .u_breakaway       = 44.0f,
        .kick_ramp         = 0.5f,
        .u_run             = 32.0f,
        .k_run             = 0.097f,
        .i_zone            = 38.0f,
        .pwm_max           = 100.0f,
        .pwm_slew_per_tick = 100.0f,
        .stall_pwm         = 80.0f,
        .stall_ms          = 500U,
    },
    .estimator = {
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
        .rest_mm                      = DB_POSE_ESTIMATOR_REST_MM,
        .free_spin_mm                 = DB_POSE_ESTIMATOR_FREE_SPIN_MM,
    },
    .steering = {
        .lever_mm              = DB_LH2_LEVER_ARM_EFFECTIVE,
        .v_max_mm_s            = DB_STEERING_V_MAX_MM_S,
        .approach_per_s        = DB_STEERING_APPROACH_PER_S,
        .runon_s               = DB_STEERING_RUNON_S,
        .spin_mm_s             = DB_STEERING_SPIN_MM_S,
        .spin_min_mm_s         = DB_STEERING_SPIN_MIN_MM_S,
        .heading_kp            = DB_STEERING_HEADING_KP,
        .heading_kd            = DB_STEERING_HEADING_KD,
        .align_enter_deg       = DB_STEERING_ALIGN_ENTER_DEG,
        .align_exit_deg        = DB_STEERING_ALIGN_EXIT_DEG,
        .full_speed_deg        = DB_STEERING_FULL_SPEED_DEG,
        .final_tol_deg         = DB_STEERING_FINAL_TOL_DEG,
        .near_mm               = DB_STEERING_NEAR_MM,
        .bearing_min_mm        = DB_STEERING_BEARING_MIN_MM,
        .lookahead_s           = DB_STEERING_LOOKAHEAD_S,
        .arrival_min_mm        = DB_STEERING_ARRIVAL_MIN_MM,
        .precise_min_mm        = DB_STEERING_PRECISE_MIN_MM,
        .pass_mm               = DB_STEERING_PASS_MM,
        .creep_mm_s            = DB_STEERING_CREEP_MM_S,
        .settle_skip_ticks     = DB_STEERING_SETTLE_SKIP_TICKS,
        .settle_fixes          = DB_STEERING_SETTLE_FIXES,
        .settle_ticks          = DB_STEERING_SETTLE_TICKS,
        .settle_nudges         = DB_STEERING_SETTLE_NUDGES,
        .nudge_ticks           = DB_STEERING_NUDGE_TICKS,
        .no_heading_turn_ticks = DB_STEERING_NO_HEADING_TURN_TICKS,
        .no_heading_ticks      = DB_STEERING_NO_HEADING_TICKS,
        .turn_ticks            = DB_STEERING_TURN_TICKS,
        .progress_ticks        = DB_STEERING_PROGRESS_TICKS,
        .progress_mm           = DB_STEERING_PROGRESS_MM,
        .hold_ticks            = DB_STEERING_HOLD_TICKS,
        .recover               = DB_STEERING_RECOVER_DRIVE,
        .recover_mm            = DB_STEERING_RECOVER_MM,
        .recover_mm_s          = DB_STEERING_RECOVER_MM_S,
        .bounds_mm             = { 0, 0, 10000.0f, 10000.0f },  // the LH2 calibration's validity rectangle
        .bounds_margin_mm      = DB_STEERING_BOUNDS_MARGIN_MM,
        .no_heading_retries    = DB_STEERING_NO_HEADING_RETRIES,
        .no_heading_rest_ticks = DB_STEERING_NO_HEADING_REST_TICKS,
    },
    .deadman_ticks = 52U,  // ~520 ms
};

//=========================== private ==========================================

/// Elapsed rather than a multiple, since the caller may step over ticks
static inline bool _due(uint32_t *last, uint32_t tick, uint32_t period) {
    if (tick - *last < period) {
        return false;
    }
    *last = tick;
    return true;
}

static void _write(db_control_output_t *out, int8_t left, int8_t right, bool brake_left, bool brake_right) {
    out->pwm_left    = left;
    out->pwm_right   = right;
    out->brake_left  = brake_left;
    out->brake_right = brake_right;
    out->write       = true;
}

/// Hands the motors to a new writer: the wheel loop starts from zero and the
/// steering drops its target unless it is the new writer
static void _enter_drive_mode(db_control_t *control, db_control_drive_mode_t mode) {
    if (mode != DB_CONTROL_DRIVE_WAYPOINT) {
        db_steering_stop(&control->steering);
    }
    control->steering_brake = false;
    control->drive_mode     = mode;
    db_wheel_control_reset(&control->wheel_left);
    db_wheel_control_reset(&control->wheel_right);
}

/// Records what stops a batch in progress; later commands leave the reason as it is
static void _note_abort(db_control_t *control, protocol_waypoints_abort_t reason) {
    if (db_steering_active(&control->steering)) {
        control->abort_reason = reason;
    }
}

/// Brakes both motors; the wheel loop releases each one once its wheel stands
static void _drive_stop(db_control_t *control, db_control_output_t *out) {
    _enter_drive_mode(control, DB_CONTROL_DRIVE_IDLE);
    _write(out, 0, 0, true, true);
}

static void _steering_pose(const db_control_t *control, db_steering_pose_t *pose) {
    switch (control->estimator.status) {
        case DB_POSE_ESTIMATOR_TRACKING:
            pose->status = DB_STEERING_POSE_TRACKING;
            break;
        case DB_POSE_ESTIMATOR_LOST:
            pose->status = DB_STEERING_POSE_LOST;
            break;
        default:
            pose->status = DB_STEERING_POSE_SEEDING;
            break;
    }
    pose->x_mm        = control->estimator.x;
    pose->y_mm        = control->estimator.y;
    pose->heading_deg = control->estimator.theta * 180.0f / (float)M_PI;
    pose->free_spin   = control->estimator.status == DB_POSE_ESTIMATOR_LOST && control->estimator.unseeded;
}

/// A brake from the steering holds both motors shorted until it asks otherwise
static void _steering_apply(db_control_t *control, const db_steering_output_t *out) {
    if (out->brake) {
        if (!control->steering_brake) {
            db_wheel_control_reset(&control->wheel_left);
            db_wheel_control_reset(&control->wheel_right);
            control->steering_brake = true;
        }
        return;
    }
    control->steering_brake = false;
    // Past the wheel limit, both wheels give up the excess, so the turn is kept
    float left   = out->left_mm_s;
    float right  = out->right_mm_s;
    float excess = fmaxf(fabsf(left), fabsf(right)) - WHEEL_SPEED_MAX_MM_S;
    if (excess > 0) {
        float shift = (left + right >= 0) ? excess : -excess;
        left -= shift;
        right -= shift;
    }
    left  = fmaxf(-WHEEL_SPEED_MAX_MM_S, fminf(WHEEL_SPEED_MAX_MM_S, left));
    right = fmaxf(-WHEEL_SPEED_MAX_MM_S, fminf(WHEEL_SPEED_MAX_MM_S, right));
    db_wheel_control_set_setpoint(&control->wheel_left, left);
    db_wheel_control_set_setpoint(&control->wheel_right, right);
}

static uint32_t _advert_period_ticks(uint32_t min_tx_interval_us) {
    uint32_t period_ms = ADVERT_PERIOD_DEF_MS;
    if (min_tx_interval_us > 0) {
        period_ms = (min_tx_interval_us / 1000U) * 100U / ADVERT_TX_SHARE_PERCENT;
        if (period_ms < ADVERT_PERIOD_MIN_MS) {
            period_ms = ADVERT_PERIOD_MIN_MS;
        } else if (period_ms > ADVERT_PERIOD_MAX_MS) {
            period_ms = ADVERT_PERIOD_MAX_MS;
        }
    }
    return period_ms / DB_CONTROL_TICK_MS;
}

static void _put(uint8_t *buf, size_t *length, const void *value, size_t size) {
    memcpy(&buf[*length], value, size);
    *length += size;
}

//=========================== public ===========================================

void db_control_init(db_control_t *control, const db_control_conf_t *conf) {
    memset(control, 0, sizeof(*control));
    control->conf = conf;
    db_wheel_control_init(&control->wheel_left, &conf->wheel);
    db_wheel_control_init(&control->wheel_right, &conf->wheel);
    db_pose_estimator_init(&control->estimator, &conf->estimator);
    db_steering_init(&control->steering, &conf->steering);
    db_lh2_fusion_init(&control->fusion);
    control->drive_mode          = DB_CONTROL_DRIVE_IDLE;
    control->abort_reason        = DB_WAYPOINTS_ABORT_STOP;
    control->advert_period_ticks = _advert_period_ticks(0);
}

void db_control_seed(db_control_t *control, float axle_x_mm, float axle_y_mm, float heading_deg) {
    db_pose_estimator_seed(&control->estimator, axle_x_mm, axle_y_mm, heading_deg, SEED_HEADING_SD_DEG);
}

void db_control_rx(db_control_t *control, const uint8_t *packet, size_t length) {
    if (length == 0 || length > DB_CONTROL_RX_MAX_BYTES) {
        return;
    }
    if (packet[0] == DB_PROTOCOL_CMD_MOVE_RAW || packet[0] == DB_PROTOCOL_CMD_WHEEL_VELOCITY || packet[0] == DB_PROTOCOL_LH2_WAYPOINTS) {
        control->last_command_tick = control->tick;
    }

    const uint8_t *payload = &packet[1];
    switch (packet[0]) {
        case DB_PROTOCOL_CMD_MOVE_RAW:
        {
            if (length < 1 + sizeof(protocol_move_raw_command_t)) {
                break;
            }
            protocol_move_raw_command_t command;
            memcpy(&command, payload, sizeof(command));
            _note_abort(control, DB_WAYPOINTS_ABORT_DIRECT);
            _enter_drive_mode(control, DB_CONTROL_DRIVE_RAW);
            _write(&control->pending, (int8_t)(100 * ((float)command.left_y / INT8_MAX)), (int8_t)(100 * ((float)command.right_y / INT8_MAX)), false, false);
        } break;
        case DB_PROTOCOL_CMD_WHEEL_VELOCITY:
        {
            if (length < 1 + sizeof(protocol_wheel_velocity_command_t)) {
                break;
            }
            protocol_wheel_velocity_command_t command;
            memcpy(&command, payload, sizeof(command));
            if (control->drive_mode != DB_CONTROL_DRIVE_VELOCITY) {
                _note_abort(control, DB_WAYPOINTS_ABORT_DIRECT);
                _enter_drive_mode(control, DB_CONTROL_DRIVE_VELOCITY);
            }
            int16_t left  = command.left_mm_s;
            int16_t right = command.right_mm_s;
            left          = (left > WHEEL_SPEED_MAX_MM_S) ? WHEEL_SPEED_MAX_MM_S : ((left < -WHEEL_SPEED_MAX_MM_S) ? -WHEEL_SPEED_MAX_MM_S : left);
            right         = (right > WHEEL_SPEED_MAX_MM_S) ? WHEEL_SPEED_MAX_MM_S : ((right < -WHEEL_SPEED_MAX_MM_S) ? -WHEEL_SPEED_MAX_MM_S : right);
            db_wheel_control_set_setpoint(&control->wheel_left, left);
            db_wheel_control_set_setpoint(&control->wheel_right, right);
        } break;
        case DB_PROTOCOL_LH2_WAYPOINTS:
        {
            db_steering_path_t path;
            uint8_t            batch_id;
            if (!db_steering_path_from_wire(payload, length - 1, &path, &batch_id)) {
                break;
            }
            // A resent batch the robot already has, its advertisement not yet heard
            if (batch_id != 0 && batch_id == control->batch_id) {
                break;
            }
            control->batch_id = batch_id;
            if (path.count == 0) {
                _note_abort(control, DB_WAYPOINTS_ABORT_STOP);
                _drive_stop(control, &control->pending);
                break;
            }
            if (control->drive_mode != DB_CONTROL_DRIVE_WAYPOINT) {
                _enter_drive_mode(control, DB_CONTROL_DRIVE_WAYPOINT);
            }
            control->steering_brake = false;
            db_steering_set_path(&control->steering, &path);
        } break;
        case DB_PROTOCOL_CMD_MAX_SPEED:
        {
            if (length < 1 + sizeof(protocol_max_speed_command_t)) {
                break;
            }
            protocol_max_speed_command_t command;
            memcpy(&command, payload, sizeof(command));
            uint16_t v = command.max_speed_mm_s;
            if (v != 0) {
                v = (v < MAX_SPEED_MIN_MM_S) ? MAX_SPEED_MIN_MM_S : ((v > MAX_SPEED_MAX_MM_S) ? MAX_SPEED_MAX_MM_S : v);
            }
            db_steering_set_max_speed(&control->steering, (float)v);
        } break;
        case DB_PROTOCOL_CONTROL_MODE:
            _note_abort(control, DB_WAYPOINTS_ABORT_CONTROL_MODE);
            _drive_stop(control, &control->pending);
            break;
        default:
            break;
    }
}

void db_control_lines(db_control_t *control, const db_lh2_floor_line_t *lines, uint8_t count) {
    if (count > DB_CONTROL_LINES_MAX) {
        count = DB_CONTROL_LINES_MAX;
    }
    memcpy(control->lines, lines, count * sizeof(lines[0]));
    control->line_count = count;
}

bool db_control_fix_due(const db_control_t *control, uint32_t elapsed_ticks) {
    uint32_t tick = control->tick + (elapsed_ticks ? elapsed_ticks : 1U);
    return tick - control->tick_position >= TICKS_PER_POSITION;
}

void db_control_tick(db_control_t *control, const db_control_input_t *in, db_control_output_t *out) {
    uint32_t elapsed = in->elapsed_ticks ? in->elapsed_ticks : 1U;
    uint32_t tick    = control->tick + elapsed;
    control->tick    = tick;

    *out                   = control->pending;
    out->advertise         = false;
    control->pending.write = false;
    control->encoder_left += (uint32_t)in->counts_left;
    control->encoder_right += (uint32_t)in->counts_right;

    // The wheel loop writes the motors while it owns them: driving by
    // velocity, and after a stop, where its zero setpoints brake the wheels
    // until they stand
    if (control->steering_brake) {
        _write(out, 0, 0, true, true);
    } else if (control->drive_mode != DB_CONTROL_DRIVE_RAW) {
        int8_t pwm_left  = db_wheel_control_step(&control->wheel_left, in->counts_left, elapsed);
        int8_t pwm_right = db_wheel_control_step(&control->wheel_right, in->counts_right, elapsed);
        _write(out, pwm_left, pwm_right, control->wheel_left.brake, control->wheel_right.brake);
    }

    db_pose_estimator_predict(&control->estimator, in->counts_left, in->counts_right, elapsed);

    bool lines_accepted = control->line_count > 0;
    for (uint8_t i = 0; i < control->line_count; i++) {
        if (db_lh2_fusion_update(&control->fusion, &control->estimator, &control->lines[i]) != DB_POSE_ESTIMATOR_ACCEPTED) {
            lines_accepted = false;
        }
    }
    control->line_count = 0;
    if (lines_accepted) {
        // As an accepted fix does: the pose is confirmed, no kidnap is building
        control->estimator.kidnap_count = 0;
        control->estimator.chain_count  = 0;
    }

    // An unchanged sequence is the previous solve read a second time
    if (_due(&control->tick_position, tick, TICKS_PER_POSITION) && in->fix_sequence != control->fix_sequence) {
        control->fix_sequence = in->fix_sequence;
        if (in->fix_x <= POSITION_INVALID_MM && in->fix_y <= POSITION_INVALID_MM) {
            control->position_x   = in->fix_x;
            control->position_y   = in->fix_y;
            control->has_position = true;
            if (!lines_accepted) {
                db_pose_estimator_update(&control->estimator, (float)in->fix_x, (float)in->fix_y);
            }
            db_steering_fix(&control->steering, (float)in->fix_x, (float)in->fix_y);
        }
    }

    uint32_t steering_elapsed = tick - control->tick_steering;
    if (_due(&control->tick_steering, tick, DB_STEERING_PERIOD_TICKS) && control->drive_mode == DB_CONTROL_DRIVE_WAYPOINT) {
        db_steering_pose_t   pose;
        db_steering_output_t steer;
        _steering_pose(control, &pose);
        db_steering_step(&control->steering, &pose, steering_elapsed, &steer);
        _steering_apply(control, &steer);
    }

    // Every tick, for the stops of a precise arrival that fall between steps
    if (control->drive_mode == DB_CONTROL_DRIVE_WAYPOINT) {
        db_steering_pose_t   pose;
        db_steering_output_t steer;
        _steering_pose(control, &pose);
        if (db_steering_poll(&control->steering, &pose, &steer)) {
            _steering_apply(control, &steer);
        }
    }

    // Raw and velocity driving both stop when the host goes silent. A waypoint
    // needs no resending: the steering stops on arrival, on losing its pose and
    // on its own timeouts.
    if (_due(&control->tick_timeout, tick, TICKS_PER_TIMEOUT) && control->drive_mode != DB_CONTROL_DRIVE_IDLE && control->drive_mode != DB_CONTROL_DRIVE_WAYPOINT && tick - control->last_command_tick > control->conf->deadman_ticks) {
        _drive_stop(control, out);
    }

    out->advertise = _due(&control->tick_advert, tick, control->advert_period_ticks);

    if (out->write) {
        control->pwm_left    = out->brake_left ? 0 : out->pwm_left;
        control->pwm_right   = out->brake_right ? 0 : out->pwm_right;
        control->brake_left  = out->brake_left;
        control->brake_right = out->brake_right;
    }
}

void db_control_set_min_tx_interval(db_control_t *control, uint32_t min_tx_interval_us) {
    control->advert_period_ticks = _advert_period_ticks(min_tx_interval_us);
}

void db_control_report(const db_control_t *control, db_control_report_t *report) {
    const db_pose_estimator_t *est = &control->estimator;
    const db_steering_t       *st  = &control->steering;
    memset(report, 0, sizeof(*report));

    report->axle_x_mm      = est->x;
    report->axle_y_mm      = est->y;
    report->heading_deg    = est->theta * 180.0f / (float)M_PI;
    report->max_speed_mm_s = st->v_max_mm_s;

    // The photodiode position: the estimator's while it tracks, else the last solve
    report->sensor_x = control->has_position ? control->position_x : 0;
    report->sensor_y = control->has_position ? control->position_y : 0;
    float sensor_x;
    float sensor_y;
    if (db_pose_estimator_sensor(est, &sensor_x, &sensor_y) && sensor_x >= 0 && sensor_y >= 0) {
        report->sensor_x = (uint32_t)lroundf(sensor_x);
        report->sensor_y = (uint32_t)lroundf(sensor_y);
    }
    if (st->state != DB_STEERING_IDLE) {
        report->waypoint_x = (uint32_t)lroundf(st->target.x_mm);
        report->waypoint_y = (uint32_t)lroundf(st->target.y_mm);
    }
    report->encoder_left  = control->encoder_left;
    report->encoder_right = control->encoder_right;
    report->fix_sequence  = control->fix_sequence;

    report->direction = DB_CONTROL_DIRECTION_INVALID;
    float heading;
    if (db_pose_estimator_heading_deg(est, &heading)) {
        report->direction = (int16_t)lroundf(heading);
    }
    report->axle_x = DB_AXLE_UNKNOWN;
    report->axle_y = DB_AXLE_UNKNOWN;
    if (est->status == DB_POSE_ESTIMATOR_TRACKING && est->x >= 0 && est->y >= 0 && est->x < DB_AXLE_UNKNOWN && est->y < DB_AXLE_UNKNOWN) {
        report->axle_x = (uint16_t)lroundf(est->x);
        report->axle_y = (uint16_t)lroundf(est->y);
    }

    report->pwm_left         = control->pwm_left;
    report->pwm_right        = control->pwm_right;
    report->brake_left       = control->brake_left;
    report->brake_right      = control->brake_right;
    report->control_mode     = (uint8_t)(db_steering_active(st) ? ControlAuto : ControlManual);
    report->drive_mode       = (uint8_t)control->drive_mode;
    report->steering_state   = (uint8_t)st->state;
    report->estimator_status = (uint8_t)est->status;
    report->waypoint_index   = st->index;
    report->waypoint_count   = st->path.count;
    report->batch_id         = control->batch_id;
    report->max_speed_10mm   = (uint8_t)lroundf(st->v_max_mm_s / 10.0f);

    switch (st->completion) {
        case DB_STEERING_DONE_IN_PROGRESS:
            report->status = DB_WAYPOINTS_IN_PROGRESS;
            break;
        case DB_STEERING_DONE_ARRIVED:
            report->status = DB_WAYPOINTS_ARRIVED;
            break;
        case DB_STEERING_DONE_FAILED:
            report->status = DB_WAYPOINTS_FAILED;
            report->reason = (uint8_t)st->fail;
            break;
        case DB_STEERING_DONE_ABORTED:
            report->status = DB_WAYPOINTS_ABORTED;
            report->reason = (uint8_t)control->abort_reason;
            break;
        default:
            report->status = DB_WAYPOINTS_NONE;
            break;
    }
}

/// Layout of DB_PROTOCOL_DOTBOT_ADVERTISEMENT, then the waypoint report; the
/// fields up to the waypoint index are the standard DotBot advertisement.
/// Fields the app does not own carry their unknown-value sentinels.
size_t db_control_advertisement(db_control_t *control, uint16_t battery_level, uint8_t *buffer) {
    db_control_report_t report;
    db_control_report(control, &report);

    size_t length       = 0;
    buffer[length++]    = DB_PROTOCOL_DOTBOT_ADVERTISEMENT;
    uint16_t calibrated = UINT16_MAX;  // calibrated station mask, unknown
    _put(buffer, &length, &calibrated, sizeof(calibrated));
    _put(buffer, &length, &report.direction, sizeof(report.direction));
    protocol_lh2_location_t position = { .x = report.sensor_x, .y = report.sensor_y };
    _put(buffer, &length, &position, sizeof(position));
    _put(buffer, &length, &battery_level, sizeof(battery_level));
    buffer[length++] = (uint8_t)report.pwm_left;
    buffer[length++] = (uint8_t)report.pwm_right;
    buffer[length++] = report.control_mode;

    int32_t encoder_left          = (int32_t)(control->encoder_left - control->advert_encoder_left);
    int32_t encoder_right         = (int32_t)(control->encoder_right - control->advert_encoder_right);
    control->advert_encoder_left  = control->encoder_left;
    control->advert_encoder_right = control->encoder_right;
    _put(buffer, &length, &encoder_left, sizeof(encoder_left));
    _put(buffer, &length, &encoder_right, sizeof(encoder_right));

    // The point being driven to, and its index; the count once arrived
    _put(buffer, &length, &report.waypoint_x, sizeof(report.waypoint_x));
    _put(buffer, &length, &report.waypoint_y, sizeof(report.waypoint_y));
    buffer[length++] = report.waypoint_index;

    protocol_waypoints_report_t waypoints = {
        .status         = report.status,
        .reason         = report.reason,
        .batch_id       = report.batch_id,
        .max_speed_10mm = report.max_speed_10mm,
        .axle_x         = report.axle_x,
        .axle_y         = report.axle_y,
    };
    _put(buffer, &length, &waypoints, sizeof(waypoints));
    return length;
}
