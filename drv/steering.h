#ifndef __STEERING_H
#define __STEERING_H

/**
 * @defgroup    drv_steering    Steering to a single target
 * @ingroup     drv
 * @brief       Outer loop: estimated pose and one target in, wheel speeds out
 *
 * Runs at the outer period on top of the wheel loop and never writes PWM. An
 * approach profile maps the distance still to go to a forward speed, a heading
 * PD gives the turn rate, and the two are mixed into wheel speeds over the
 * effective track (db_wheel_control_from_twist()). There is no velocity floor
 * and no outer integral, so forward speed falls to zero with the heading error
 * and the robot turns in place.
 *
 * The target is a photodiode position, the point the LH2 fixes and the
 * advertisement give. With no final heading the photodiode itself is steered
 * onto it: the axle is aimed at the target and driven until the photodiode,
 * a lever arm ahead, reaches it. With a final heading the axle is steered to
 * the point a lever arm behind the target along that heading, then the robot
 * turns in place, which leaves the photodiode on the target.
 *
 * Errors are taken from the pose predicted lookahead_s ahead along the last
 * command, which covers the wheel lag. The arrival test uses the point the robot would stop at if braked
 * now, run-on included.
 *
 * Within near_mm of the lever arm, a target behind the axle is reached by
 * backing up rather than by turning round: the heading error folds into
 * [-90, 90] and the forward speed takes the sign of the along-track error.
 *
 * States: IDLE until a target is set. NO_HEADING spins in place at spin_mm_s
 * until the pose is tracking. ALIGN turns in place until the heading error is
 * below align_exit_deg, DRIVE drives with heading correction and returns to
 * ALIGN above align_enter_deg. FINAL_TURN turns in place to the final heading.
 * ARRIVED latches a stop. HOLD stops while the pose is LOST and resumes once
 * it tracks again. FAILED latches a stop on a timeout, on the heading being
 * lost mid-move, or on HOLD running out. ARRIVED, HOLD and FAILED ask for the
 * motors to be braked.
 *
 * Headings are in degrees, 0 facing +y and positive clockwise, so body-forward
 * is (-sin, +cos), as in the pose estimator.
 *
 * No hardware calls, so the module also builds on the host for its tests.
 *
 * @{
 * @file
 * @copyright Inria, 2026
 * @}
 */

#include <stdbool.h>
#include <stdint.h>

//=========================== defines ==========================================

/// Period of one scheduler tick, the unit of the elapsed_ticks argument
#define DB_STEERING_TICK_MS (10U)

/// Outer period in scheduler ticks: 100 ms, one LH2 fix
#define DB_STEERING_PERIOD_TICKS (10U)

/// Cruise speed, mm/s: the fastest straight the estimator was floor-tested at
/// (300 mm/s, about 2 mm of lag after the fix-age fix)
#define DB_STEERING_V_MAX_MM_S (300.0f)

/// Slope of the approach profile, mm/s per mm to go: the robot covers 40 per
/// cent of what is left per 100 ms period, well inside run-on plus one period
#define DB_STEERING_APPROACH_PER_S (4.0f)

/// Run-on after braking from a speed, in s of that speed (wheel loop contract:
/// 3.4 mm from 100 mm/s, 8.7 from 200, 15 from 300)
#define DB_STEERING_RUNON_S (0.05f)

/// Largest turn, as the difference of each wheel from the mean, mm/s: the
/// trusted spin limit, about 280 deg/s on the 81 to 83 mm spin track
#define DB_STEERING_SPIN_MM_S (200.0f)

/// Least turn while turning in place with the error above tolerance, per wheel,
/// mm/s: the wheel loop holds 15 mm/s within 5 per cent and runs pulsed below
#define DB_STEERING_SPIN_MIN_MM_S (20.0f)

/// Heading P gain, deg/s per deg of error: half the error per period
#define DB_STEERING_HEADING_KP (5.0f)

/// Heading D gain, deg/s per deg/s of error rate
#define DB_STEERING_HEADING_KD (0.05f)

/// Heading error above which DRIVE stops and turns in place, deg
#define DB_STEERING_ALIGN_ENTER_DEG (20.0f)

/// Heading error below which ALIGN starts to drive, deg
#define DB_STEERING_ALIGN_EXIT_DEG (8.0f)

/// Heading error up to which DRIVE keeps its full approach speed, deg; the speed
/// then falls linearly to zero at DB_STEERING_ALIGN_ENTER_DEG
#define DB_STEERING_FULL_SPEED_DEG (5.0f)

/// Final heading tolerance, deg: about twice the estimator's heading error at rest
#define DB_STEERING_FINAL_TOL_DEG (3.0f)

/// Margin beyond the lever arm within which a target behind is backed up to, mm:
/// an overshoot of the photodiode by up to about 150 mm is reversed out of
#define DB_STEERING_NEAR_MM (50.0f)

/// Distance from the axle below which the bearing is not steered on, mm
#define DB_STEERING_BEARING_MIN_MM (5.0f)

/// Pose prediction ahead of the estimate, s: the wheel lag, the same 50 ms that
/// sets the run-on. A command is held for the whole period, which the gains
/// already allow for, so predicting a full period over-leads and slows the approach.
#define DB_STEERING_LOOKAHEAD_S (0.05f)

/// Smallest arrival threshold honoured, mm: about twice the estimator's RMS
/// position error while driving
#define DB_STEERING_ARRIVAL_MIN_MM (5.0f)

/// Longest NO_HEADING spin before giving up, ticks: two turns at the spin limit
#define DB_STEERING_NO_HEADING_TICKS (300U)

/// Longest turn in place in ALIGN or FINAL_TURN, ticks
#define DB_STEERING_TURN_TICKS (300U)

/// Time allowed to get DB_STEERING_PROGRESS_MM closer, ticks
#define DB_STEERING_PROGRESS_TICKS (300U)

/// Progress that resets the progress timeout, mm
#define DB_STEERING_PROGRESS_MM (5.0f)

/// Longest HOLD before giving up, ticks
#define DB_STEERING_HOLD_TICKS (500U)

/// Steering state
typedef enum {
    DB_STEERING_IDLE,        ///< No target
    DB_STEERING_NO_HEADING,  ///< Spinning in place until the pose tracks
    DB_STEERING_ALIGN,       ///< Turning in place toward the target
    DB_STEERING_DRIVE,       ///< Driving with heading correction
    DB_STEERING_FINAL_TURN,  ///< Turning in place to the final heading
    DB_STEERING_ARRIVED,     ///< Stopped at the target, latched
    DB_STEERING_HOLD,        ///< Stopped while the pose is lost
    DB_STEERING_FAILED,      ///< Stopped for good, latched
} db_steering_state_t;

/// Why a move ended in FAILED
typedef enum {
    DB_STEERING_FAIL_NONE,          ///< Not failed
    DB_STEERING_FAIL_NO_HEADING,    ///< No heading after the NO_HEADING spin
    DB_STEERING_FAIL_TURN,          ///< A turn in place took too long
    DB_STEERING_FAIL_PROGRESS,      ///< Stopped getting closer
    DB_STEERING_FAIL_HEADING_LOST,  ///< Heading lost mid-move, as after a kidnap
    DB_STEERING_FAIL_HOLD,          ///< The pose stayed lost too long
} db_steering_fail_t;

/// Pose status, as the estimator reports it
typedef enum {
    DB_STEERING_POSE_TRACKING,  ///< Pose and heading valid
    DB_STEERING_POSE_SEEDING,   ///< No heading
    DB_STEERING_POSE_LOST,      ///< No fix for a while; pose predicted only
} db_steering_pose_status_t;

/// Estimated pose
typedef struct {
    db_steering_pose_status_t status;       ///< validity
    float                     x_mm;         ///< axle midpoint
    float                     y_mm;         ///< axle midpoint
    float                     heading_deg;  ///< 0 along +y, clockwise positive
} db_steering_pose_t;

/// One target
typedef struct {
    float x_mm;               ///< photodiode position to reach
    float y_mm;               ///< photodiode position to reach
    float threshold_mm;       ///< arrival radius
    bool  has_final_heading;  ///< turn to final_heading_deg once there
    float final_heading_deg;  ///< 0 along +y, clockwise positive
} db_steering_target_t;

/// Gains and limits; the app holds one, the steering keeps a pointer to it
typedef struct {
    float    lever_mm;          ///< axle midpoint to photodiode, mm
    float    v_max_mm_s;        ///< cruise speed, mm/s
    float    approach_per_s;    ///< approach profile slope, mm/s per mm
    float    runon_s;           ///< braking run-on, s of speed
    float    spin_mm_s;         ///< turn cap, per wheel from the mean, mm/s
    float    spin_min_mm_s;     ///< least turn in place, per wheel, mm/s
    float    heading_kp;        ///< deg/s per deg
    float    heading_kd;        ///< deg/s per deg/s
    float    align_enter_deg;   ///< DRIVE to ALIGN above this error, deg
    float    align_exit_deg;    ///< ALIGN to DRIVE below this error, deg
    float    full_speed_deg;    ///< full approach speed up to this error, deg
    float    final_tol_deg;     ///< final heading tolerance, deg
    float    near_mm;           ///< back-up margin beyond the lever arm, mm
    float    bearing_min_mm;    ///< no bearing steering closer than this, mm
    float    lookahead_s;       ///< pose prediction, s
    float    arrival_min_mm;    ///< least arrival threshold, mm
    uint32_t no_heading_ticks;  ///< NO_HEADING timeout
    uint32_t turn_ticks;        ///< ALIGN and FINAL_TURN timeout
    uint32_t progress_ticks;    ///< progress timeout
    float    progress_mm;       ///< progress that resets it, mm
    uint32_t hold_ticks;        ///< HOLD timeout
} db_steering_conf_t;

/// Steering state
typedef struct {
    const db_steering_conf_t *conf;            ///< gains, not owned
    db_steering_state_t       state;           ///< current state
    db_steering_fail_t        fail;            ///< why FAILED, else NONE
    db_steering_target_t      target;          ///< current target
    float                     v_mm_s;          ///< last commanded forward speed
    float                     omega_deg_s;     ///< last commanded turn rate
    float                     error_deg;       ///< last heading error
    bool                      has_error;       ///< error_deg is valid for the D term
    float                     distance_mm;     ///< last distance of the steered point to its goal
    float                     best_mm;         ///< closest the steered point has been since the timer reset
    uint32_t                  state_ticks;     ///< ticks in the current state
    uint32_t                  progress_ticks;  ///< ticks since best_mm last improved
} db_steering_t;

/// Wheel speeds to apply
typedef struct {
    float left_mm_s;   ///< left wheel setpoint
    float right_mm_s;  ///< right wheel setpoint
    bool  brake;       ///< stop and hold the motors braked, ignoring the setpoints
} db_steering_output_t;

//=========================== prototypes =======================================

/**
 * @brief   Bind the steering to its gains and go IDLE
 *
 * @param[out]  steering    Steering state
 * @param[in]   conf        Gains, which must outlive the steering
 */
void db_steering_init(db_steering_t *steering, const db_steering_conf_t *conf);

/**
 * @brief   Start a move to a target, replacing any move in progress
 *
 * A move already driving carries on driving toward the new target.
 *
 * @param[in]   steering    Steering state
 * @param[in]   target      Target, copied
 */
void db_steering_set_target(db_steering_t *steering, const db_steering_target_t *target);

/**
 * @brief   Drop the target and go IDLE
 *
 * @param[in]   steering    Steering state
 */
void db_steering_stop(db_steering_t *steering);

/**
 * @brief   Run one outer step
 *
 * @param[in]   steering        Steering state
 * @param[in]   pose            Estimated pose
 * @param[in]   elapsed_ticks   Scheduler ticks since the previous step
 * @param[out]  out             Wheel speeds to apply until the next step
 */
void db_steering_step(db_steering_t *steering, const db_steering_pose_t *pose, uint32_t elapsed_ticks, db_steering_output_t *out);

/**
 * @brief   Whether a move is in progress, the advertisement's automatic mode
 *
 * @param[in]   steering    Steering state
 *
 * @return  true from NO_HEADING to HOLD
 */
bool db_steering_active(const db_steering_t *steering);

#endif
