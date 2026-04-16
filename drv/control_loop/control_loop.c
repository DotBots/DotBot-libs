#include <stdio.h>
#include <string.h>
#include <math.h>
#ifdef DOTBOT_SIMULATION
#include <stdlib.h>
#endif
#include "protocol.h"
#include "control_loop.h"

#if defined(BOARD_DOTBOT_V3)
#define DB_MAX_PWM             (60)     ///< Max speed in autonomous control mode
#define DB_REDUCE_SPEED_FACTOR (0.75f)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE  (25)     ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SIDE_FACTOR (-1)     ///< Angular side factor
#define DB_ANGULAR_SPEED_GAIN  (1.0f)
#elif defined(BOARD_DOTBOT_V2)
#define DB_MAX_PWM             (70)    ///< Max speed in autonomous control mode
#define DB_REDUCE_SPEED_FACTOR (0.8f)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE  (25)    ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SIDE_FACTOR (-1)    ///< Angular side factor
#define DB_ANGULAR_SPEED_GAIN  (0.6f)
#else                                  // BOARD_DOTBOT_V1
#define DB_MAX_PWM             (70)    ///< Max speed in autonomous control mode
#define DB_REDUCE_SPEED_FACTOR (0.9f)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE  (20)    ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SIDE_FACTOR (1)     ///< Angular side factor
#define DB_ANGULAR_SPEED_GAIN  (0.6f)
#endif

// Encoder odometry constant: mm of wheel travel per encoder count.
// Formula: pi * wheel_diameter_mm / (counts_per_rev * gear_ratio)
// = pi * 50 / (12 * 50)
#define MM_PER_COUNT (0.26180f)

// EKF physical parameter
#define EKF_L (70.0f)  ///< Distance between the two wheels in mm (wheelbase)

// EKF tuning — process noise Q (diagonal)
#define EKF_Q_POS   (10.0f)   ///< Position process noise variance (mm²)
#define EKF_Q_THETA (0.001f)  ///< Heading process noise variance (rad²)

// EKF tuning — measurement noise R (diagonal, LH2 accuracy)
#define EKF_R_POS   (25.0f)  ///< Position measurement noise variance (mm²)
#define EKF_R_THETA (0.05f)  ///< Heading measurement noise variance (rad²)

// EKF predict sanity cap: heading change larger than this per step is likely
// due to wheel slip or encoder counts accumulated over multiple missed LH2
// periods.  Clamp to prevent heading divergence; LH2 update corrects residual
// error on the next measurement.  ~86° in radians.
#define EKF_MAX_DDIR (1.5f)

// EKF initial covariance (set equal to R since we seed from the first measurement)
#define EKF_P0_POS   EKF_R_POS
#define EKF_P0_THETA EKF_R_THETA

/// Internal control loop state — opaque to all callers.
typedef struct {
    // Navigation
    coordinate_t waypoints[DB_MAX_WAYPOINTS];
    uint8_t      waypoints_length;
    uint8_t      waypoint_idx;
    uint32_t     waypoint_threshold;
    // EKF state: [x_mm, y_mm, theta_rad]
    // theta uses the same convention as `direction`: 0 = north, positive = clockwise
    float ekf_x[3];   ///< State estimate [x, y, theta]
    float ekf_P[9];   ///< 3x3 covariance matrix (row-major)
    bool  ekf_ready;  ///< True once the filter has been seeded from the first LH2 fix
} control_loop_state_t;

#ifdef DOTBOT_SIMULATION
void *control_loop_alloc(void) {
    return calloc(1, sizeof(control_loop_state_t));
}

void control_loop_free(void *ctx) {
    free(ctx);
}
#else
static control_loop_state_t _state = { 0 };

void *control_loop_alloc(void) {
    return &_state;
}

void control_loop_free(void *ctx) {
    (void)ctx;
}
#endif

void control_loop_set_waypoints(void *ctx, const coordinate_t *waypoints, uint8_t count, uint32_t threshold) {
    control_loop_state_t *state = (control_loop_state_t *)ctx;
    if (count > DB_MAX_WAYPOINTS) {
        count = DB_MAX_WAYPOINTS;
    }
    memcpy(state->waypoints, waypoints, count * sizeof(coordinate_t));
    state->waypoints_length   = count;
    state->waypoint_threshold = threshold;
    state->waypoint_idx       = 0;
    // EKF state is NOT reset here: the robot's physical state is continuous across
    // navigation commands; only the target sequence changes.
}

//=========================== EKF helpers ======================================

/// Normalise an angle to [-pi, pi].
static float _wrap_angle(float a) {
    a = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
    if (a < 0.0f) {
        a += 2.0f * (float)M_PI;
    }
    return a - (float)M_PI;
}

/// C = A * B  (3x3 row-major matrices).
static void _mat3_mul(const float a[9], const float b[9], float c[9]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c[i * 3 + j] = a[i * 3 + 0] * b[0 * 3 + j] +
                           a[i * 3 + 1] * b[1 * 3 + j] +
                           a[i * 3 + 2] * b[2 * 3 + j];
        }
    }
}

/// Ainv = A^{-1}  via the cofactor/adjugate method.  Returns false if singular.
static bool _mat3_inv(const float a[9], float ainv[9]) {
    float det = a[0] * (a[4] * a[8] - a[5] * a[7]) -
                a[1] * (a[3] * a[8] - a[5] * a[6]) +
                a[2] * (a[3] * a[7] - a[4] * a[6]);
    if (fabsf(det) < 1e-10f) {
        return false;
    }
    float inv_det = 1.0f / det;
    ainv[0]       = (a[4] * a[8] - a[5] * a[7]) * inv_det;
    ainv[1]       = (a[2] * a[7] - a[1] * a[8]) * inv_det;
    ainv[2]       = (a[1] * a[5] - a[2] * a[4]) * inv_det;
    ainv[3]       = (a[5] * a[6] - a[3] * a[8]) * inv_det;
    ainv[4]       = (a[0] * a[8] - a[2] * a[6]) * inv_det;
    ainv[5]       = (a[2] * a[3] - a[0] * a[5]) * inv_det;
    ainv[6]       = (a[3] * a[7] - a[4] * a[6]) * inv_det;
    ainv[7]       = (a[1] * a[6] - a[0] * a[7]) * inv_det;
    ainv[8]       = (a[0] * a[4] - a[1] * a[3]) * inv_det;
    return true;
}

/// EKF predict step: propagate state and covariance using encoder odometry.
/// enc_left / enc_right are signed delta counts since the last call.
///
/// Coordinate convention (screen space, matches the DotBot system):
///   x increases to the right, y increases downward.
///   ekf_x[2] stores direction in radians: 0 = facing down (+y), CW positive.
///   direction = -theta  where theta is the simulator's internal integration angle.
///
/// Kinematics in direction convention:
///   dx = -d · sin(dir)   [e.g. dir=-90° (right): dx = +d ✓]
///   dy = +d · cos(dir)   [e.g. dir=  0° (down):  dy = +d ✓]
///   ddir = -(d_right - d_left) / L   [left faster → CW → dir increases ✓]
static void _ekf_predict(control_loop_state_t *state, int32_t enc_left, int32_t enc_right) {
    float d_left  = (float)enc_left * MM_PER_COUNT;
    float d_right = (float)enc_right * MM_PER_COUNT;
    float d       = (d_left + d_right) * 0.5f;
    // Change in direction (rad): positive = CW = left wheel faster
    float ddir = -(d_right - d_left) / EKF_L;
    // Clamp to guard against wheel slip or accumulated counts over missed LH2 periods
    if (ddir > EKF_MAX_DDIR) {
        ddir = EKF_MAX_DDIR;
    } else if (ddir < -EKF_MAX_DDIR) {
        ddir = -EKF_MAX_DDIR;
    }

    // Use midpoint direction to reduce linearisation error
    float dir_mid = state->ekf_x[2] + ddir * 0.5f;
    float cos_mid = cosf(dir_mid);
    float sin_mid = sinf(dir_mid);

    // State update
    state->ekf_x[0] += -d * sin_mid;
    state->ekf_x[1] += d * cos_mid;
    state->ekf_x[2] = _wrap_angle(state->ekf_x[2] + ddir);

    // Covariance predict: P = F * P * F^T + Q
    // Motion Jacobian (∂f/∂[x, y, dir]):
    //   F = [[1, 0, -d·cos_mid],
    //        [0, 1, -d·sin_mid],
    //        [0, 0,  1        ]]
    float *P = state->ekf_P;
    float  fp[9];  // F * P

    fp[0] = P[0] - d * cos_mid * P[6];
    fp[1] = P[1] - d * cos_mid * P[7];
    fp[2] = P[2] - d * cos_mid * P[8];
    fp[3] = P[3] - d * sin_mid * P[6];
    fp[4] = P[4] - d * sin_mid * P[7];
    fp[5] = P[5] - d * sin_mid * P[8];
    fp[6] = P[6];
    fp[7] = P[7];
    fp[8] = P[8];

    // (F * P) * F^T — F^T = [[1, 0, 0], [0, 1, 0], [-d·cos, -d·sin, 1]]
    // Only columns 0 and 1 of F*P*F^T differ from F*P; column 2 is unchanged.
    for (int i = 0; i < 3; i++) {
        P[i * 3 + 0] = fp[i * 3 + 0] - fp[i * 3 + 2] * d * cos_mid;
        P[i * 3 + 1] = fp[i * 3 + 1] - fp[i * 3 + 2] * d * sin_mid;
        P[i * 3 + 2] = fp[i * 3 + 2];
    }

    // Add diagonal process noise Q
    P[0] += EKF_Q_POS;
    P[4] += EKF_Q_POS;
    P[8] += EKF_Q_THETA;
}

/// EKF update step: correct state and covariance with an LH2 position+heading measurement.
/// meas_theta is in radians, same convention as ekf_x[2].
static void _ekf_update(control_loop_state_t *state, float meas_x, float meas_y, float meas_theta) {
    float *P = state->ekf_P;

    // S = P + R  (H = I, so H*P*H^T = P)
    float s[9];
    memcpy(s, P, 9 * sizeof(float));
    s[0] += EKF_R_POS;
    s[4] += EKF_R_POS;
    s[8] += EKF_R_THETA;

    float s_inv[9];
    if (!_mat3_inv(s, s_inv)) {
        return;  // Numerically singular — skip update, keep predict result
    }

    // K = P * S^{-1}
    float k[9];
    _mat3_mul(P, s_inv, k);

    // Innovation (angle wrapped to avoid discontinuity near ±180°)
    float innov[3] = {
        meas_x - state->ekf_x[0],
        meas_y - state->ekf_x[1],
        _wrap_angle(meas_theta - state->ekf_x[2]),
    };

    // State correction:  x += K * innov
    for (int i = 0; i < 3; i++) {
        state->ekf_x[i] += k[i * 3 + 0] * innov[0] +
                           k[i * 3 + 1] * innov[1] +
                           k[i * 3 + 2] * innov[2];
    }
    state->ekf_x[2] = _wrap_angle(state->ekf_x[2]);

    // Covariance correction:  P = (I - K) * P
    float ik[9];
    memcpy(ik, k, 9 * sizeof(float));
    for (int i = 0; i < 9; i++) {
        ik[i] = -ik[i];
    }
    ik[0] += 1.0f;
    ik[4] += 1.0f;
    ik[8] += 1.0f;

    float p_new[9];
    _mat3_mul(ik, P, p_new);
    memcpy(P, p_new, 9 * sizeof(float));
}

//=========================== public API =======================================

bool compute_angle(const coordinate_t *origin, const coordinate_t *next, int16_t *angle) {
    float dx       = (float)next->x - (float)origin->x;
    float dy       = (float)next->y - (float)origin->y;
    float distance = sqrtf(powf(dx, 2) + powf(dy, 2));

    *angle = (int16_t)(atan2f(dx, dy) * -1 * 180 / M_PI);  // atan2f returns angle in radians in [-PI, PI], converted here to degrees in [-180, 180] with 0 being north and positive angles being clockwise
    return distance > DB_DIRECTION_THRESHOLD;
}

void update_control(robot_control_t *control, void *ctx) {
    control_loop_state_t *state = (control_loop_state_t *)ctx;

    // Clear status flags at the start of each call
    control->waypoint_reached = 0;
    control->all_done         = 0;

    if (state->waypoints_length == 0 || state->waypoint_idx >= state->waypoints_length) {
        control->pwm_left  = 0;
        control->pwm_right = 0;
        return;
    }

    if (control->direction == DB_DIRECTION_INVALID) {
        // Heading unknown — move forward until the first valid LH2 fix
        control->pwm_left  = (int16_t)DB_MAX_PWM;
        control->pwm_right = (int16_t)DB_MAX_PWM;
        return;
    }

    float meas_theta = (float)control->direction * (float)M_PI / 180.0f;

    if (!state->ekf_ready) {
        // Seed the filter from the first valid LH2 measurement
        state->ekf_x[0] = (float)control->pos_x;
        state->ekf_x[1] = (float)control->pos_y;
        state->ekf_x[2] = meas_theta;
        memset(state->ekf_P, 0, sizeof(state->ekf_P));
        state->ekf_P[0]  = EKF_P0_POS;
        state->ekf_P[4]  = EKF_P0_POS;
        state->ekf_P[8]  = EKF_P0_THETA;
        state->ekf_ready = true;
    } else {
        _ekf_predict(state, control->encoder_left, control->encoder_right);
        _ekf_update(state, (float)control->pos_x, (float)control->pos_y, meas_theta);
    }

    // All subsequent calculations use the EKF-filtered estimates
    float   est_x         = state->ekf_x[0];
    float   est_y         = state->ekf_x[1];
    int16_t est_direction = (int16_t)(state->ekf_x[2] * 180.0f / (float)M_PI);

    // Publish current target to the I/O struct for telemetry
    control->waypoint_idx = state->waypoint_idx;
    control->waypoint_x   = state->waypoints[state->waypoint_idx].x;
    control->waypoint_y   = state->waypoints[state->waypoint_idx].y;

    float dx                 = (float)control->waypoint_x - est_x;
    float dy                 = (float)control->waypoint_y - est_y;
    float distance_to_target = sqrtf(dx * dx + dy * dy);

    if (distance_to_target < (float)state->waypoint_threshold) {
        // Target waypoint is reached
        control->waypoint_reached = 1;
        state->waypoint_idx++;
        control->waypoint_idx = state->waypoint_idx;
        if (state->waypoint_idx >= state->waypoints_length) {
            control->pwm_left  = 0;
            control->pwm_right = 0;
            control->all_done  = 1;
        }
        return;
    }

    coordinate_t next            = { .x = control->waypoint_x, .y = control->waypoint_y };
    coordinate_t origin          = { .x = (uint32_t)est_x, .y = (uint32_t)est_y };
    int16_t      angle_to_target = 0;
    if (!compute_angle(&origin, &next, &angle_to_target)) {
        angle_to_target = 0;
    }

    // Normalise direction to [-180, 180) to avoid wrap-around in error computation
    int16_t direction = est_direction;
    if (direction >= 180) {
        direction -= 360;
    } else if (direction < -180) {
        direction += 360;
    }

    int16_t error_angle = angle_to_target - direction;
    if (error_angle >= 180) {
        error_angle -= 360;
    } else if (error_angle < -180) {
        error_angle += 360;
    }

    float speed_reduction_factor = 1.0f;
    if (distance_to_target < (float)(state->waypoint_threshold * 3)) {
        speed_reduction_factor = DB_REDUCE_SPEED_FACTOR;
    }
    if (error_angle > DB_REDUCE_SPEED_ANGLE || error_angle < -DB_REDUCE_SPEED_ANGLE) {
        speed_reduction_factor = DB_REDUCE_SPEED_FACTOR;
    }

    float angular_speed = (float)(error_angle / 180.0f) * DB_MAX_PWM * DB_ANGULAR_SIDE_FACTOR * DB_ANGULAR_SPEED_GAIN;
    control->pwm_left   = (int16_t)((DB_MAX_PWM * speed_reduction_factor) - angular_speed);
    control->pwm_right  = (int16_t)((DB_MAX_PWM * speed_reduction_factor) + angular_speed);
}
