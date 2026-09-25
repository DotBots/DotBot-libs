#ifndef __STEERING_H
#define __STEERING_H

/**
 * @defgroup    drv_steering    Steering along waypoints
 * @ingroup     drv
 * @brief       Outer loop: estimated pose and a batch of waypoints in, wheel speeds out
 *
 * Runs at the outer period on top of the wheel loop and never writes PWM. An
 * approach profile maps the distance still to go to a forward speed, a heading
 * PD gives the turn rate, and the two are mixed into wheel speeds over the
 * effective track (db_wheel_control_from_twist()). There is no outer integral,
 * so forward speed falls to zero with the heading error and the robot turns in
 * place.
 *
 * The points are driven in order, each one a position for the axle midpoint,
 * the robot's centre. An intermediate point is passed without stopping once
 * the axle is within the pass radius or past it along the leg; the approach
 * keeps the speed the turn onto the next leg allows, full up to
 * full_speed_deg and none from align_enter_deg, where the robot turns in
 * place. The last one is arrived at within the threshold and braked, latched.
 * A threshold below arrival_min_mm is precise: the robot creeps in, stops,
 * takes the axle from the fixes averaged at rest, and creeps along its heading,
 * or first turns in place to point it at the goal, until that axle is within
 * the threshold. A point with a heading is a pose: the robot turns in place to
 * the heading there, and goes on to the next point or, at the last, arrives.
 * The photodiode, a lever arm ahead of the axle, is only what LH2 sees.
 *
 * Errors are taken from the pose predicted lookahead_s ahead along the last
 * command, which covers the wheel lag. The arrival test uses the point the
 * robot would stop at if braked now, run-on included.
 *
 * Within near_mm of the axle, a target behind it is reached by
 * backing up rather than by turning round: the heading error folds into
 * [-90, 90] and the forward speed takes the sign of the along-track error.
 *
 * States: IDLE until a batch is set. NO_HEADING, entered without a heading,
 * spins in place at spin_mm_s for one full turn and on until the pose tracks.
 * ALIGN turns in place until the heading error is below align_exit_deg, DRIVE drives with heading correction and returns to
 * ALIGN above align_enter_deg. FINAL_TURN turns in place to a pose's heading.
 * SETTLE and NUDGE are the precise arrival's rest and correction. ARRIVED
 * latches a stop. HOLD stops while the pose is LOST and resumes once
 * it tracks again. FAILED latches a stop on a timeout, on the heading being
 * lost mid-move and not re-acquired, or on HOLD running out. A heading lost
 * while moving (the pose SEEDING) is re-acquired by RECOVER, a straight of up
 * to recover_mm along the last heading, when recover is DRIVE and the straight
 * stays inside the bounds; otherwise by NO_HEADING's spin. ARRIVED, HOLD,
 * SETTLE and FAILED ask for the motors to be braked.
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
#include <stddef.h>
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

/// Distance from the axle within which a target behind is backed up to, mm
#define DB_STEERING_NEAR_MM (100.0f)

/// Distance from the axle below which the bearing is not steered on, mm
#define DB_STEERING_BEARING_MIN_MM (5.0f)

/// Pose prediction ahead of the estimate, s: the wheel lag, the same 50 ms that
/// sets the run-on. A command is held for the whole period, which the gains
/// already allow for, so predicting a full period over-leads and slows the approach.
#define DB_STEERING_LOOKAHEAD_S (0.05f)

/// Smallest arrival threshold reached by driving alone, mm: about twice the
/// estimator's RMS position error while driving. Below it the arrival is precise.
#define DB_STEERING_ARRIVAL_MIN_MM (5.0f)

/// Smallest precise threshold honoured, mm
#define DB_STEERING_PRECISE_MIN_MM (0.5f)

/// Pass radius of an intermediate point when the batch gives none, mm
#define DB_STEERING_PASS_MM (20.0f)

/// Slowest the precise approach and its nudges drive, per wheel, mm/s: the
/// wheel loop holds 15 mm/s within 5 per cent
#define DB_STEERING_CREEP_MM_S (20.0f)

/// Rest before the fixes of a precise arrival are averaged, ticks: the brake
/// and the 2-tick fix age
#define DB_STEERING_SETTLE_SKIP_TICKS (20U)

/// Fixes averaged at rest per precise check
#define DB_STEERING_SETTLE_FIXES (4U)

/// Longest wait for them, ticks
#define DB_STEERING_SETTLE_TICKS (150U)

/// Corrections a precise arrival may make before it fails
#define DB_STEERING_SETTLE_NUDGES (6U)

/// Longest correction, ticks
#define DB_STEERING_NUDGE_TICKS (100U)

/// Most points in a batch
#define DB_STEERING_MAX_POINTS (16U)

/// NO_HEADING spin, ticks: one full turn at the spin limit on the 82 mm spin
/// track, 2 pi x 41 mm / 200 mm/s = 1.29 s. One turn leaves the heading within
/// about 1 degree at rest, against about 4 at the moment it is first acquired.
#define DB_STEERING_NO_HEADING_TURN_TICKS (130U)

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

/// Straight travel allowed to re-acquire a heading lost while moving, mm: the
/// estimator acquires after 40 to 44 mm of straight travel
#define DB_STEERING_RECOVER_MM (60.0f)

/// Speed of that straight, mm/s
#define DB_STEERING_RECOVER_MM_S (120.0f)

/// Distance the photodiode must stay inside the bounds after the recovery straight, mm
#define DB_STEERING_BOUNDS_MARGIN_MM (100.0f)

/// How a heading lost while moving is re-acquired
typedef enum {
    DB_STEERING_RECOVER_SPIN,   ///< Stop and spin a full turn, as from rest
    DB_STEERING_RECOVER_DRIVE,  ///< Drive straight on along the last heading
} db_steering_recover_t;

/**
 * @brief   Steering state
 *
 * @verbatim
 *          set_path               tracks (1)           |err| < align_exit_deg
 * IDLE ------------> NO_HEADING ------------> ALIGN -----------------------> DRIVE
 *                                               |   <-----------------------   |
 *                                               |    |err| > align_enter_deg   |
 *                                               +--------------+---------------+
 *                                                              | stop point within threshold (2)
 *                                      at a pose               v      at the last point
 *                      FINAL_TURN <--------------------------- + ---------------------> ARRIVED
 *                          |                                                            ^  ^
 *                          +----------- heading within tolerance (3), last point -------+  |
 *                                                                                          |
 *                      DRIVE ----------> SETTLE ---- mean of settle_fixes within (4) ------+
 *                           stop point    |  ^
 *                           near (4)      |  |  correction done, or after nudge_ticks
 *           off, under settle_nudges (4)  v  |
 *                                        NUDGE
 *
 * (1) at once if the pose already tracks, else after no_heading_turn_ticks of spin
 * (2) max(threshold, arrival_min_mm), where the axle stops if braked now; not when precise
 * (3) heading_tol_deg, or final_tol_deg when 0
 * (4) precise: the last point, no heading, threshold under arrival_min_mm. Within
 *     max(threshold, precise_min_mm); near is within half of it, or where it passes closest
 *
 * batch         intermediate point passed (within pass_mm, or beyond it along its leg): next
 *               point, same state; FINAL_TURN within tolerance, more points: next point, ALIGN
 * pose SEEDING  in ALIGN, DRIVE, FINAL_TURN, SETTLE, NUDGE: RECOVER when moving, recover is
 *               DRIVE and the straight stays inside bounds_mm; else NO_HEADING
 * pose LOST     in NO_HEADING, ALIGN, DRIVE, FINAL_TURN, SETTLE, NUDGE, RECOVER: HOLD
 * HOLD          tracks: ALIGN; SEEDING: NO_HEADING
 * RECOVER       tracks: ALIGN
 *
 * to FAILED     NO_HEADING after no_heading_ticks                         fail NO_HEADING
 *               ALIGN, FINAL_TURN after turn_ticks                        fail TURN
 *               ALIGN, DRIVE not progress_mm closer in progress_ticks     fail PROGRESS
 *               RECOVER after recover_mm, still SEEDING                   fail HEADING_LOST
 *               HOLD after hold_ticks, still LOST                         fail HOLD
 *               SETTLE off after settle_nudges, or fixes not in by        fail SETTLE
 *               settle_ticks
 *
 * set_path      IDLE, ARRIVED, FAILED, HOLD, RECOVER: NO_HEADING; ALIGN, FINAL_TURN, SETTLE,
 *               NUDGE: ALIGN; DRIVE and NO_HEADING stay; an empty batch is a stop
 * stop          any: IDLE
 *
 * completion    IN_PROGRESS on set_path; ARRIVED, FAILED on entering them; ABORTED on a
 *               stop while active
 * @endverbatim
 */
typedef enum {
    DB_STEERING_IDLE,        ///< No target
    DB_STEERING_NO_HEADING,  ///< Spinning in place until the pose tracks
    DB_STEERING_ALIGN,       ///< Turning in place toward the target
    DB_STEERING_DRIVE,       ///< Driving with heading correction
    DB_STEERING_FINAL_TURN,  ///< Turning in place to the final heading
    DB_STEERING_ARRIVED,     ///< Stopped at the target, latched
    DB_STEERING_HOLD,        ///< Stopped while the pose is lost
    DB_STEERING_FAILED,      ///< Stopped for good, latched
    DB_STEERING_RECOVER,     ///< Driving straight on to re-acquire a heading lost while moving
    DB_STEERING_SETTLE,      ///< Precise arrival: braked, averaging the fixes at rest
    DB_STEERING_NUDGE,       ///< Precise arrival: a small correction along the heading or in place
} db_steering_state_t;

/// How the batch stands
typedef enum {
    DB_STEERING_DONE_NONE,         ///< No batch since init
    DB_STEERING_DONE_IN_PROGRESS,  ///< Moving, turning, settling or holding
    DB_STEERING_DONE_ARRIVED,      ///< At the last point
    DB_STEERING_DONE_FAILED,       ///< Gave up, see fail
    DB_STEERING_DONE_ABORTED,      ///< Stopped by db_steering_stop() while in progress
} db_steering_completion_t;

/// Why a move ended in FAILED
typedef enum {
    DB_STEERING_FAIL_NONE,          ///< Not failed
    DB_STEERING_FAIL_NO_HEADING,    ///< No heading after the NO_HEADING spin
    DB_STEERING_FAIL_TURN,          ///< A turn in place took too long
    DB_STEERING_FAIL_PROGRESS,      ///< Stopped getting closer
    DB_STEERING_FAIL_HEADING_LOST,  ///< Heading not re-acquired after it was lost mid-move
    DB_STEERING_FAIL_HOLD,          ///< The pose stayed lost too long
    DB_STEERING_FAIL_SETTLE,        ///< A precise arrival did not settle within its threshold
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
    float x_mm;               ///< axle midpoint position to reach
    float y_mm;               ///< axle midpoint position to reach
    float threshold_mm;       ///< arrival radius
    bool  has_final_heading;  ///< turn to final_heading_deg once there
    float final_heading_deg;  ///< 0 along +y, clockwise positive
} db_steering_target_t;

/// One point of a batch
typedef struct {
    float x_mm;         ///< axle midpoint position
    float y_mm;         ///< axle midpoint position
    bool  has_heading;  ///< a pose: turn to heading_deg there
    float heading_deg;  ///< 0 along +y, clockwise positive
} db_steering_point_t;

/// A batch of points, driven in order
typedef struct {
    db_steering_point_t points[DB_STEERING_MAX_POINTS];  ///< in order
    uint8_t             count;                           ///< points used, at least 1
    float               threshold_mm;                    ///< arrival radius at the last point and at poses
    float               pass_mm;                         ///< pass radius of intermediate points; 0 for the conf's
    float               heading_tol_deg;                 ///< tolerance of pose headings; 0 for the conf's
} db_steering_path_t;

/// Gains and limits; the app holds one, the steering keeps a pointer to it
typedef struct {
    float                 lever_mm;               ///< axle midpoint to photodiode, mm
    float                 v_max_mm_s;             ///< cruise speed, mm/s
    float                 approach_per_s;         ///< approach profile slope, mm/s per mm
    float                 runon_s;                ///< braking run-on, s of speed
    float                 spin_mm_s;              ///< turn cap, per wheel from the mean, mm/s
    float                 spin_min_mm_s;          ///< least turn in place, per wheel, mm/s
    float                 heading_kp;             ///< deg/s per deg
    float                 heading_kd;             ///< deg/s per deg/s
    float                 align_enter_deg;        ///< DRIVE to ALIGN above this error, deg
    float                 align_exit_deg;         ///< ALIGN to DRIVE below this error, deg
    float                 full_speed_deg;         ///< full approach speed up to this error, deg
    float                 final_tol_deg;          ///< final heading tolerance, deg
    float                 near_mm;                ///< back-up distance from the axle, mm
    float                 bearing_min_mm;         ///< no bearing steering closer than this, mm
    float                 lookahead_s;            ///< pose prediction, s
    float                 arrival_min_mm;         ///< least threshold reached by driving, mm; below it precise
    float                 precise_min_mm;         ///< least precise threshold, mm
    float                 pass_mm;                ///< default pass radius, mm
    float                 creep_mm_s;             ///< precise approach and nudge speed, mm/s
    uint32_t              settle_skip_ticks;      ///< rest before averaging fixes
    uint32_t              settle_fixes;           ///< fixes averaged per check
    uint32_t              settle_ticks;           ///< longest wait for them
    uint32_t              settle_nudges;          ///< corrections before failing
    uint32_t              nudge_ticks;            ///< longest correction
    uint32_t              no_heading_turn_ticks;  ///< NO_HEADING spin, one full turn
    uint32_t              no_heading_ticks;       ///< NO_HEADING timeout
    uint32_t              turn_ticks;             ///< ALIGN and FINAL_TURN timeout
    uint32_t              progress_ticks;         ///< progress timeout
    float                 progress_mm;            ///< progress that resets it, mm
    uint32_t              hold_ticks;             ///< HOLD timeout
    db_steering_recover_t recover;                ///< re-acquisition of a heading lost while moving
    float                 recover_mm;             ///< straight travel allowed for it, mm
    float                 recover_mm_s;           ///< speed of that straight, mm/s
    float                 bounds_mm[4];           ///< x0, y0, x1, y1 the recovery straight must stay inside; all 0 for none
    float                 bounds_margin_mm;       ///< margin inside the bounds, mm
} db_steering_conf_t;

/// Steering state
typedef struct {
    const db_steering_conf_t *conf;               ///< gains, not owned
    db_steering_state_t       state;              ///< current state
    db_steering_fail_t        fail;               ///< why the last batch FAILED, else NONE
    db_steering_completion_t  completion;         ///< how the batch stands
    db_steering_path_t        path;               ///< the batch
    uint8_t                   index;              ///< point being driven to; count once arrived
    db_steering_target_t      target;             ///< current point, as a target
    float                     v_max_mm_s;         ///< cruise speed in force
    float                     start_x_mm;         ///< axle where the batch started, the first leg's start
    float                     start_y_mm;         ///< axle where the batch started, the first leg's start
    bool                      has_start;          ///< the two above are valid
    float                     settle_x_mm;        ///< sum of the fixes at rest
    float                     settle_y_mm;        ///< sum of the fixes at rest
    uint32_t                  settle_count;       ///< fixes summed
    uint32_t                  nudges;             ///< corrections made
    bool                      nudge_turn;         ///< the correction turns in place, else drives along the heading
    float                     nudge_goal;         ///< mm along the heading, or deg of turn, signed
    float                     nudge_done;         ///< progress so far, same unit
    float                     nudge_rate;         ///< progress per s, from the last poll
    float                     nudge_x_mm;         ///< axle at the start of the correction
    float                     nudge_y_mm;         ///< axle at the start of the correction
    float                     nudge_heading_deg;  ///< heading at the start of the correction
    float                     v_mm_s;             ///< last commanded forward speed
    float                     omega_deg_s;        ///< last commanded turn rate
    float                     error_deg;          ///< last heading error
    bool                      has_error;          ///< error_deg is valid for the D term
    float                     distance_mm;        ///< last distance of the steered point to its goal
    float                     best_mm;            ///< closest the steered point has been since the timer reset
    uint32_t                  state_ticks;        ///< ticks in the current state
    bool                      spinning;           ///< NO_HEADING has started its turn
    float                     last_x_mm;          ///< photodiode at the last tracking step
    float                     last_y_mm;          ///< photodiode at the last tracking step
    float                     last_heading_deg;   ///< heading at the last tracking step
    bool                      has_last;           ///< the three above are valid
    float                     recover_sign;       ///< +1 forward, -1 backward, during RECOVER
    uint32_t                  progress_ticks;     ///< ticks since best_mm last improved
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
 * The same as a batch of one point. A move already driving carries on
 * driving toward the new target.
 *
 * @param[in]   steering    Steering state
 * @param[in]   target      Target, copied
 */
void db_steering_set_target(db_steering_t *steering, const db_steering_target_t *target);

/**
 * @brief   Start a batch, replacing any move in progress
 *
 * A move already driving carries on driving toward the first point.
 *
 * @param[in]   steering    Steering state
 * @param[in]   path        Batch, copied; count 0 is db_steering_stop(), and points past DB_STEERING_MAX_POINTS are dropped
 */
void db_steering_set_path(db_steering_t *steering, const db_steering_path_t *path);

/**
 * @brief   Read a DB_PROTOCOL_LH2_WAYPOINTS payload into a batch
 *
 * Threshold, count and points, then the trailer if the payload holds all of
 * it: batch id, heading tolerance, pass radius, and a heading per point.
 * Without it, no point has a heading and the batch id is 0. Points past
 * DB_STEERING_MAX_POINTS are dropped.
 *
 * @param[in]   payload     Bytes after the type byte
 * @param[in]   length      Their number
 * @param[out]  path        Batch; count 0 is a stop
 * @param[out]  batch_id    The trailer's batch id, 0 without one
 *
 * @return  false if the payload is shorter than its count of points
 */
bool db_steering_path_from_wire(const uint8_t *payload, size_t length, db_steering_path_t *path, uint8_t *batch_id);

/**
 * @brief   Set the cruise speed, from the next step on
 *
 * @param[in]   steering    Steering state
 * @param[in]   v_mm_s      Cruise speed, mm/s; 0 for the conf's
 */
void db_steering_set_max_speed(db_steering_t *steering, float v_mm_s);

/**
 * @brief   Drop the batch and go IDLE; a batch in progress is ABORTED
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
 * @brief   Check a precise arrival between steps, on the newest pose
 *
 * Call it every scheduler tick. The creep in and the corrections are short,
 * and a whole period is 2 mm at the creep speed.
 *
 * @param[in]   steering    Steering state
 * @param[in]   pose        Estimated pose
 * @param[out]  out         Wheel speeds, written only when it returns true
 *
 * @return  true if the robot must stop now
 */
bool db_steering_poll(db_steering_t *steering, const db_steering_pose_t *pose, db_steering_output_t *out);

/**
 * @brief   Hand the steering a raw fix, which a precise arrival averages at rest
 *
 * @param[in]   steering    Steering state
 * @param[in]   x_mm        photodiode fix
 * @param[in]   y_mm        photodiode fix
 */
void db_steering_fix(db_steering_t *steering, float x_mm, float y_mm);

/**
 * @brief   Whether a move is in progress, the advertisement's automatic mode
 *
 * @param[in]   steering    Steering state
 *
 * @return  true from NO_HEADING to HOLD
 */
bool db_steering_active(const db_steering_t *steering);

#endif
