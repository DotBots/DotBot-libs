#ifndef __CONTROL_LOOP_H
#define __CONTROL_LOOP_H

/**
 * @defgroup    drv_control_loop  Control loop implementation for the DotBot
 * @ingroup     drv
 * @brief       Functions for computing the control loop of the DotBot
 *
 * The functions compute the PWM values to apply to the motors based on the current position, direction, and target waypoint
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2026
 * @}
 */

#include <stdint.h>
#include <stdbool.h>

#define DB_DIRECTION_INVALID   (-1000)  ///< Invalid angle e.g out of [0, 360] range
#define DB_DIRECTION_THRESHOLD (50)     ///< Threshold to update the direction (50mm)

/// Coordinate struct
typedef struct {
    uint32_t x;  ///< X coordinate in mm
    uint32_t y;  ///< Y coordinate in mm
} coordinate_t;

/// Robot control struct
typedef struct {
    // Inputs, robot state
    uint32_t pos_x;  ///< X coordinate of the robot in mm
    uint32_t pos_y;  ///< Y coordinate of the robot in mm
    int32_t  vel_x;  ///< X velocity of the robot in mm/s (from encoders; 0 if unavailable)
    int32_t  vel_y;  ///< Y velocity of the robot in mm/s (from encoders; 0 if unavailable)
    uint32_t dt_us;  ///< Time elapsed since last call in microseconds (for integrators / PID)
    // Inputs, current waypoint
    uint32_t waypoint_x;          ///< X coordinate of the current target waypoint in mm
    uint32_t waypoint_y;          ///< Y coordinate of the current target waypoint in mm
    uint32_t waypoint_threshold;  ///< Distance threshold to consider a waypoint reached, in mm
    int16_t  direction;           ///< Direction of the robot in degrees, in [0, 360], with 0 being north and positive angles being clockwise
    uint8_t  waypoints_length;    ///< Number of waypoints in the waypoints array
    uint8_t  waypoint_idx;        ///< Index of the current target waypoint (written by C)
    // Outputs, actuation
    int8_t pwm_left;   ///< PWM value for the left motor, in [-DB_MAX_PWM, DB_MAX_PWM]
    int8_t pwm_right;  ///< PWM value for the right motor, in [-DB_MAX_PWM, DB_MAX_PWM]
    // Outputs, status flags (written by C, read by caller)
    uint8_t waypoint_reached;  ///< Set to 1 by C when the current waypoint is reached, cleared on next call
    uint8_t all_done;          ///< Set to 1 by C when all waypoints in the batch are completed
} robot_control_t;

/**
 * @brief Compute the angle between the robot's current position and a target position
 *
 * @param origin    Pointer to the robot's current coordinate
 * @param next      Pointer to the target coordinate
 * @param angle     Pointer to the variable where the computed angle will be stored, in degrees, in [0, 360], with 0 being north and positive angles being clockwise
 */
bool compute_angle(const coordinate_t *origin, const coordinate_t *next, int16_t *angle);

/**
 * @brief Update the robot's control variables based on the current position, direction, and target waypoint
 *
 * @param control   Pointer to the robot_control_t struct containing the current control variables. The pwm_left and pwm_right fields will be updated by this function.
 */
void update_control(robot_control_t *control);

#endif  // __CONTROL_LOOP_H
