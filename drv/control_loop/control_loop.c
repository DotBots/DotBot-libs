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

/// Internal control loop state — opaque to all callers.
typedef struct {
    coordinate_t waypoints[DB_MAX_WAYPOINTS];
    uint8_t      waypoints_length;
    uint8_t      waypoint_idx;
    uint32_t     waypoint_threshold;
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
}

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

    // Publish current target to the I/O struct for telemetry
    control->waypoint_idx = state->waypoint_idx;
    control->waypoint_x   = state->waypoints[state->waypoint_idx].x;
    control->waypoint_y   = state->waypoints[state->waypoint_idx].y;

    float dx                 = (float)control->waypoint_x - (float)control->pos_x;
    float dy                 = (float)control->waypoint_y - (float)control->pos_y;
    float distance_to_target = sqrtf(powf(dx, 2) + powf(dy, 2));

    if ((uint32_t)(distance_to_target) < state->waypoint_threshold) {
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

    if (control->direction == DB_DIRECTION_INVALID) {
        // Unknown direction, just move forward a bit
        control->pwm_left  = (int16_t)DB_MAX_PWM;
        control->pwm_right = (int16_t)DB_MAX_PWM;
        return;
    }

    coordinate_t next            = { .x = control->waypoint_x, .y = control->waypoint_y };
    coordinate_t origin          = { .x = control->pos_x, .y = control->pos_y };
    int16_t      angle_to_target = 0;
    if (!compute_angle(&origin, &next, &angle_to_target)) {
        angle_to_target = 0;
    }

    // Normalise direction to [-180, 180) locally to avoid issues with angle wrapping when computing the error angle
    int16_t direction = control->direction;
    if (direction >= 180) {
        direction -= 360;
    } else if (direction < -180) {
        direction += 360;
    }

    float   angular_speed = 0;
    int16_t error_angle   = 0;
    error_angle           = angle_to_target - direction;
    if (error_angle >= 180) {
        error_angle -= 360;
    } else if (error_angle < -180) {
        error_angle += 360;
    }

    float speedReductionFactor = 1.0;  // No reduction by default
    if ((uint32_t)(distance_to_target) < state->waypoint_threshold * 3) {
        speedReductionFactor = DB_REDUCE_SPEED_FACTOR;
    }

    if (error_angle > DB_REDUCE_SPEED_ANGLE || error_angle < -DB_REDUCE_SPEED_ANGLE) {
        speedReductionFactor = DB_REDUCE_SPEED_FACTOR;
    }
    angular_speed      = (float)(error_angle / 180.0f) * DB_MAX_PWM * DB_ANGULAR_SIDE_FACTOR * DB_ANGULAR_SPEED_GAIN;
    control->pwm_left  = (int16_t)(((DB_MAX_PWM * speedReductionFactor) - angular_speed));
    control->pwm_right = (int16_t)(((DB_MAX_PWM * speedReductionFactor) + angular_speed));

    // printf(
    //     "Loop data - direction: %i - "
    //     "dx: %f - dy: %f - distance to target: %f - "
    //     "angle to target: %i - error angle: %i - angular speed: %f - "
    //     "pwm_left: %i - pwm_right: %i\n",
    //     control->direction,
    //     distance_to_target, dx, dy,
    //     angle_to_target, error_angle, angular_speed,
    //     control->pwm_left, control->pwm_right
    // );
}
