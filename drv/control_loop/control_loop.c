#include <stdio.h>
#include <math.h>
#include "control_loop.h"

#if defined(BOARD_DOTBOT_V3)
#define DB_MAX_PWM             (60)    ///< Max speed in autonomous control mode
#define DB_REDUCE_SPEED_FACTOR (0.8f)  ///< Reduction factor applied to speed when close to target
#define DB_REDUCE_SPEED_ANGLE  (25)    ///< Angle threshold above which speed reduction is applied (degrees)
#define DB_ANGULAR_SIDE_FACTOR (-1)    ///< Angular side factor (motor wiring convention)
#define DB_ANGULAR_SPEED_GAIN  (0.6f)
#define DB_DRIVE_ANGULAR_GAIN  (1.8f)  ///< Steering gain in DRIVE phase (higher than rotate gain)
#define DB_WHEELBASE_MM        (70.0f)  ///< Distance between wheel contact points in mm
#define DB_WHEEL_DIAMETER_MM   (50.0f)  ///< Wheel diameter in mm
#define DB_ENCODER_CPR         (12)     ///< Encoder counts per motor shaft revolution
#define DB_MOTOR_REDUCTION     (50)     ///< Motor gearbox reduction ratio
#elif defined(BOARD_DOTBOT_V2)
#define DB_MAX_PWM             (70)
#define DB_REDUCE_SPEED_FACTOR (0.8f)
#define DB_REDUCE_SPEED_ANGLE  (25)
#define DB_ANGULAR_SIDE_FACTOR (-1)
#define DB_ANGULAR_SPEED_GAIN  (0.6f)
#define DB_DRIVE_ANGULAR_GAIN  (1.8f)
#define DB_WHEELBASE_MM        (90.0f)
#define DB_WHEEL_DIAMETER_MM   (40.0f)
#define DB_ENCODER_CPR         (12)
#define DB_MOTOR_REDUCTION     (50)
#else  // BOARD_DOTBOT_V1
#define DB_MAX_PWM             (70)
#define DB_REDUCE_SPEED_FACTOR (0.9f)
#define DB_REDUCE_SPEED_ANGLE  (20)
#define DB_ANGULAR_SIDE_FACTOR (1)
#define DB_ANGULAR_SPEED_GAIN  (0.6f)
#define DB_DRIVE_ANGULAR_GAIN  (1.8f)
#define DB_WHEELBASE_MM        (90.0f)
#define DB_WHEEL_DIAMETER_MM   (40.0f)
#define DB_ENCODER_CPR         (12)
#define DB_MOTOR_REDUCTION     (50)
#endif

// Navigation phase thresholds
#define DB_ROTATE_ENTER_ANGLE (45)  ///< Enter ROTATE phase when |error_angle| exceeds this (degrees)
#define DB_ROTATE_EXIT_ANGLE  (15)  ///< Exit ROTATE phase when |error_angle| drops below this (degrees)
#define DB_DRIVE_DWELL_MIN    (3)   ///< Minimum calls in DRIVE before ROTATE re-entry is allowed

// Anti-stall parameters
#define DB_STALL_PWM_MIN      (5)   ///< Min commanded PWM below which silence is expected (no boost)
#define DB_STALL_ENCODER_MIN  (2)   ///< Encoder counts below this are considered a stall
#define DB_STALL_COUNT_THRESH (3)   ///< Consecutive stall detections before boosting
#define DB_STALL_BOOST_STEP   (5)   ///< PWM boost increment per stall interval
#define DB_STALL_BOOST_MAX    (30)  ///< Maximum accumulated boost
#define DB_STALL_BOOST_DECAY  (2)   ///< Boost reduction per call when motor is moving

// Rotate phase minimum PWM — ensures the robot actually overcomes static friction while rotating
#define DB_MIN_ROTATE_PWM (35)  ///< Minimum |PWM| applied during ROTATE phase

// Speed governor
#define DB_RAMP_DISTANCE_FACTOR (4)   ///< Deceleration ramp = factor × waypoint_threshold
#define DB_MIN_FORWARD_PWM      (35)  ///< Minimum forward PWM during DRIVE to avoid stall/boost churn near waypoints

// Derived constant: mm per encoder count
#define DB_MM_PER_COUNT ((float)(M_PI * DB_WHEEL_DIAMETER_MM) / (DB_ENCODER_CPR * DB_MOTOR_REDUCTION))

// Clamp helper
static inline int16_t _clamp16(int16_t v, int16_t lo, int16_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

bool compute_angle(const coordinate_t *origin, const coordinate_t *next, int16_t *angle) {
    float dx       = (float)next->x - (float)origin->x;
    float dy       = (float)next->y - (float)origin->y;
    float distance = sqrtf(powf(dx, 2) + powf(dy, 2));

    *angle = (int16_t)(atan2f(dx, dy) * -1 * 180 / M_PI);  // atan2f returns angle in radians in [-PI, PI], converted here to degrees in [-180, 180] with 0 being north and positive angles being clockwise
    return distance > DB_DIRECTION_THRESHOLD;
}

// Snap odo_heading to the LH2 direction when it is valid.
// Called whenever we enter ROTATE from a known-good position (waypoint advance or DRIVE→ROTATE).
static void _reset_odo_heading_from_lh2(robot_control_t *control) {
    if (control->direction != DB_DIRECTION_INVALID) {
        float lh2 = (float)control->direction;
        if (lh2 >= 180.0f)
            lh2 -= 360.0f;
        control->odo_heading = lh2;
    }
}

// Wrap an angle (float) to [-180, 180)
static float _wrap180f(float a) {
    while (a >= 180.0f)
        a -= 360.0f;
    while (a < -180.0f)
        a += 360.0f;
    return a;
}

// Update odometric heading from encoder deltas, and optionally correct with LH2 direction.
// Returns the best heading estimate in [-180, 180).
static float _update_odo_heading(robot_control_t *control) {
    // Update odometric heading from encoder deltas
    float d_left  = (float)control->encoder_left * DB_MM_PER_COUNT;
    float d_right = (float)control->encoder_right * DB_MM_PER_COUNT;
    // Encoder counts are always positive=forward regardless of motor wiring, so no side factor here.
    // CW rotation: d_left<0, d_right>0 → d_theta negative (heading decreases toward east=-90°). ✓
    float d_theta        = (d_left - d_right) / DB_WHEELBASE_MM * (180.0f / (float)M_PI);
    control->odo_heading = _wrap180f(control->odo_heading + d_theta);

    // When the robot is translating (large displacement since last LH2 sample), trust the
    // LH2 direction to correct odometry drift.
    if (control->direction != DB_DIRECTION_INVALID) {
        float lh2 = (float)control->direction;
        if (lh2 >= 180.0f)
            lh2 -= 360.0f;
        // Blend toward LH2 only when the robot is actually translating forward (not rotating).
        // Use net forward displacement |(d_left + d_right) / 2| — near zero during pure rotation.
        float d_forward = fabsf((d_left + d_right) * 0.5f);
        if (d_forward > DB_DIRECTION_THRESHOLD * 0.1f) {
            // Soft correction: blend 20% toward LH2 each call when moving forward
            float err            = _wrap180f(lh2 - control->odo_heading);
            control->odo_heading = _wrap180f(control->odo_heading + 0.2f * err);
        }
    }
    return control->odo_heading;
}

// Apply per-motor anti-stall boost and return adjusted PWM (clamped to int8 range).
static int8_t _apply_boost(int8_t pwm_cmd, int32_t encoder_delta, int8_t *boost, uint8_t *stall_count) {
    int8_t abs_pwm = pwm_cmd < 0 ? -pwm_cmd : pwm_cmd;

    if (abs_pwm >= DB_STALL_PWM_MIN) {
        int32_t abs_enc = encoder_delta < 0 ? -encoder_delta : encoder_delta;
        if (abs_enc < DB_STALL_ENCODER_MIN) {
            // Stall detected
            (*stall_count)++;
            if (*stall_count >= DB_STALL_COUNT_THRESH) {
                *boost += DB_STALL_BOOST_STEP;
                if (*boost > DB_STALL_BOOST_MAX) {
                    *boost = DB_STALL_BOOST_MAX;
                }
            }
        } else {
            // Motor is moving — decay boost
            *stall_count = 0;
            if (*boost > 0) {
                *boost -= DB_STALL_BOOST_DECAY;
                if (*boost < 0)
                    *boost = 0;
            }
        }
    } else {
        // Low/zero command — don't boost, decay stall counter
        *stall_count = 0;
        if (*boost > 0) {
            *boost -= DB_STALL_BOOST_DECAY;
            if (*boost < 0)
                *boost = 0;
        }
    }

    // Apply boost in the direction of the command
    int16_t boosted = (int16_t)pwm_cmd;
    if (pwm_cmd > 0) {
        boosted += *boost;
    } else if (pwm_cmd < 0) {
        boosted -= *boost;
    }
    return (int8_t)_clamp16(boosted, -127, 127);
}

void update_control(robot_control_t *control) {
    // Clear per-call status flags
    control->waypoint_reached = 0;
    control->all_done         = 0;

    float dx                 = (float)control->waypoint_x - (float)control->pos_x;
    float dy                 = (float)control->waypoint_y - (float)control->pos_y;
    float distance_to_target = sqrtf(powf(dx, 2) + powf(dy, 2));

    if ((uint32_t)(distance_to_target) < control->waypoint_threshold) {
        // Waypoint reached — stop motors, advance index
        control->pwm_left         = 0;
        control->pwm_right        = 0;
        control->waypoint_reached = 1;
        control->waypoint_idx++;
        // Reset boost/stall state and phase for the next waypoint
        control->boost_left        = 0;
        control->boost_right       = 0;
        control->stall_count_left  = 0;
        control->stall_count_right = 0;
        control->drive_dwell       = 0;
        control->nav_state         = DB_NAV_ROTATE;
        // Snap odo_heading to LH2 so rotation starts from a correct reference
        _reset_odo_heading_from_lh2(control);
        if (control->waypoint_idx >= control->waypoints_length) {
            control->all_done = 1;
        }
        return;
    }

    // Update odometric heading
    float heading = _update_odo_heading(control);

    // Compute angle to target
    coordinate_t next            = { .x = control->waypoint_x, .y = control->waypoint_y };
    coordinate_t origin          = { .x = control->pos_x, .y = control->pos_y };
    int16_t      angle_to_target = 0;
    if (!compute_angle(&origin, &next, &angle_to_target)) {
        angle_to_target = 0;
    }

    // Error angle using odometric heading (reliable during rotation)
    float error_angle = _wrap180f((float)angle_to_target - heading);

    // Phase transitions
    if (control->nav_state == DB_NAV_DRIVE) {
        control->drive_dwell++;
        // Only re-enter ROTATE if the error is large AND the robot has had time to settle in DRIVE.
        // This prevents the min-PWM overshoot from immediately bouncing back into ROTATE.
        if (control->drive_dwell >= DB_DRIVE_DWELL_MIN &&
            (error_angle > DB_ROTATE_ENTER_ANGLE || error_angle < -DB_ROTATE_ENTER_ANGLE)) {
            control->nav_state         = DB_NAV_ROTATE;
            control->drive_dwell       = 0;
            control->boost_left        = 0;
            control->boost_right       = 0;
            control->stall_count_left  = 0;
            control->stall_count_right = 0;
            // Re-anchor odo_heading from LH2 — it's reliable when we've been translating
            _reset_odo_heading_from_lh2(control);
        }
    } else {  // DB_NAV_ROTATE
        if (error_angle <= DB_ROTATE_EXIT_ANGLE && error_angle >= -DB_ROTATE_EXIT_ANGLE) {
            control->nav_state         = DB_NAV_DRIVE;
            control->drive_dwell       = 0;
            control->boost_left        = 0;
            control->boost_right       = 0;
            control->stall_count_left  = 0;
            control->stall_count_right = 0;
        }
    }

    int8_t raw_left, raw_right;

    if (control->nav_state == DB_NAV_ROTATE) {
        // Pure in-place rotation: proportional to error angle, with a minimum magnitude
        // to ensure the robot overcomes static friction (especially on carpet).
        float angular_speed = (error_angle / 180.0f) * DB_MAX_PWM * (float)DB_ANGULAR_SIDE_FACTOR * DB_ANGULAR_SPEED_GAIN;
        // Enforce minimum magnitude to overcome static friction, but only outside the exit zone.
        // Clamp on angular_speed's own sign — it already incorporates DB_ANGULAR_SIDE_FACTOR, so
        // using error_angle's sign here would flip the direction on boards where the factor is -1.
        float abs_angular    = angular_speed < 0.0f ? -angular_speed : angular_speed;
        float exit_threshold = ((float)DB_ROTATE_EXIT_ANGLE / 180.0f) * DB_MAX_PWM * DB_ANGULAR_SPEED_GAIN;
        // if (abs_angular > exit_threshold && abs_angular < (float)DB_MIN_ROTATE_PWM) {
        //     angular_speed = angular_speed > 0.0f ? (float)DB_MIN_ROTATE_PWM : -(float)DB_MIN_ROTATE_PWM;
        // }
        raw_left  = (int8_t)_clamp16((int16_t)(-angular_speed), -DB_MAX_PWM, DB_MAX_PWM);
        raw_right = (int8_t)_clamp16((int16_t)(angular_speed), -DB_MAX_PWM, DB_MAX_PWM);
    } else {
        // DRIVE phase: forward with heading correction
        // Speed governor: linear ramp-down inside ramp distance
        float ramp_distance = (float)(control->waypoint_threshold * DB_RAMP_DISTANCE_FACTOR);
        float speed_factor  = distance_to_target / ramp_distance;
        if (speed_factor > 1.0f)
            speed_factor = 1.0f;
        // Also reduce if angle error is still non-negligible
        if (error_angle > DB_REDUCE_SPEED_ANGLE || error_angle < -DB_REDUCE_SPEED_ANGLE) {
            if (speed_factor > DB_REDUCE_SPEED_FACTOR)
                speed_factor = DB_REDUCE_SPEED_FACTOR;
        }

        float forward_pwm = DB_MAX_PWM * speed_factor;
        if (forward_pwm < DB_MIN_FORWARD_PWM)
            forward_pwm = DB_MIN_FORWARD_PWM;
        // Use a higher gain than ROTATE so steering is strong enough to curve without stopping.
        // The inner wheel is allowed to go below DB_MIN_FORWARD_PWM — the floor only applies to
        // the average speed, not per-wheel, so tight turns still work.
        float angular_speed = (error_angle / 180.0f) * DB_MAX_PWM * (float)DB_ANGULAR_SIDE_FACTOR * DB_DRIVE_ANGULAR_GAIN;
        raw_left            = (int8_t)_clamp16((int16_t)(forward_pwm - angular_speed), -DB_MAX_PWM, DB_MAX_PWM);
        raw_right           = (int8_t)_clamp16((int16_t)(forward_pwm + angular_speed), -DB_MAX_PWM, DB_MAX_PWM);
    }

    // Apply per-motor anti-stall boost
    control->pwm_left  = _apply_boost(raw_left, control->encoder_left, &control->boost_left, &control->stall_count_left);
    control->pwm_right = _apply_boost(raw_right, control->encoder_right, &control->boost_right, &control->stall_count_right);
}
