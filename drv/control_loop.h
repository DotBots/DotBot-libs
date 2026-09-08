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

/// Coordinate struct (also used as a waypoint type)
typedef struct {
    uint32_t x;  ///< X coordinate in mm
    uint32_t y;  ///< Y coordinate in mm
} coordinate_t;

/// Robot I/O struct — the stable ABI boundary between the application and the control loop.
/// Only fields that the calling application must provide or consume appear here.
/// All internal algorithm state (navigation index, thresholds, waypoint list, future EKF
/// matrices, …) lives in the opaque context allocated by control_loop_alloc().
typedef struct {
    // Inputs — robot state, refreshed on every call
    uint32_t pos_x;          ///< X coordinate of the robot in mm
    uint32_t pos_y;          ///< Y coordinate of the robot in mm
    int32_t  encoder_left;   ///< Left encoder delta counts since last call (signed; 0 if unavailable)
    int32_t  encoder_right;  ///< Right encoder delta counts since last call (signed; 0 if unavailable)
    // Outputs — current target waypoint coordinates (written by C, useful for telemetry)
    uint32_t waypoint_x;  ///< X coordinate of the current target waypoint in mm
    uint32_t waypoint_y;  ///< Y coordinate of the current target waypoint in mm
    // Input — robot heading
    int16_t direction;  ///< Direction of the robot in degrees, in [-180, 180], with 0 being north and positive angles being clockwise
    // Outputs — actuation (written by C)
    int8_t pwm_left;   ///< PWM value for the left motor, in [-DB_MAX_PWM, DB_MAX_PWM]
    int8_t pwm_right;  ///< PWM value for the right motor, in [-DB_MAX_PWM, DB_MAX_PWM]
    // Outputs — status flags (written by C, read by caller)
    uint8_t waypoint_reached;  ///< Set to 1 by C when the current waypoint is reached, cleared on next call
    uint8_t all_done;          ///< Set to 1 by C when all waypoints in the batch are completed
    uint8_t waypoint_idx;      ///< Index of the current target waypoint (written by C, useful for telemetry)
} robot_control_t;

/**
 * @brief Allocate and initialise the opaque control loop context.
 *
 * On embedded targets (DOTBOT_SIMULATION not defined) this returns a pointer to
 * a file-scope static struct — no heap allocation is performed.
 * In simulation (DOTBOT_SIMULATION defined) it uses calloc so that each
 * simulated robot gets an independent context.
 *
 * @return Opaque pointer to the internal control loop state.  Never NULL.
 */
void *control_loop_alloc(void);

/**
 * @brief Release a context previously obtained from control_loop_alloc().
 *
 * On embedded targets this is a no-op.  In simulation it calls free().
 *
 * @param ctx  Opaque context pointer returned by control_loop_alloc().
 */
void control_loop_free(void *ctx);

/**
 * @brief Load a new waypoint list into the control loop context.
 *
 * Must be called once before the first update_control() call of each
 * autonomous navigation session, and again whenever a new navigation
 * command arrives.  Resets the internal waypoint index to zero.
 *
 * @param ctx        Opaque context pointer returned by control_loop_alloc().
 * @param waypoints  Array of waypoint coordinates (copied internally).
 * @param count      Number of waypoints (clamped to DB_CONTROL_LOOP_MAX_WAYPOINTS).
 * @param threshold  Distance in mm below which a waypoint is considered reached.
 */
void control_loop_set_waypoints(void *ctx, const coordinate_t *waypoints, uint8_t count, uint32_t threshold);

/**
 * @brief Compute the angle between the robot's current position and a target position
 *
 * @param origin    Pointer to the robot's current coordinate
 * @param next      Pointer to the target coordinate
 * @param angle     Pointer to the variable where the computed angle will be stored, in degrees,
 *                  in [-180, 180] with 0 being north and positive angles being clockwise
 */
bool compute_angle(const coordinate_t *origin, const coordinate_t *next, int16_t *angle);

/**
 * @brief Update the robot's control variables based on the current position, direction, and target waypoint.
 *
 * Called at the LH2 position update rate.  The caller must fill all Input fields of
 * @p control before calling, and may read all Output fields after returning.
 *
 * @param control   Pointer to the robot_control_t I/O struct.
 * @param ctx       Opaque context pointer returned by control_loop_alloc().
 */
void update_control(robot_control_t *control, void *ctx);

/// The robot dimensions this library was compiled with, so a host that keeps
/// its own copy can check the two against each other.
typedef struct {
    float wheel_diameter_mm;    ///< Wheel diameter
    float track_mm;             ///< Distance between the two wheel mid-planes
    float encoder_cpr;          ///< Quadrature counts per motor shaft revolution
    float gear_ratio;           ///< Motor shaft revolutions per wheel revolution
    float mm_per_count;         ///< Wheel travel per encoder count
    float lh2_lever_arm_mm;     ///< Axle midpoint to the lighthouse photodiode
    float lh2_lever_angle_deg;  ///< Direction of that offset, clockwise from forward
} control_loop_geometry_t;

/**
 * @brief Read the geometry compiled into this library.
 *
 * @param geometry  Pointer to the struct to fill.
 */
void control_loop_get_geometry(control_loop_geometry_t *geometry);

#endif  // __CONTROL_LOOP_H
