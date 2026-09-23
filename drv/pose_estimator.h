#ifndef __POSE_ESTIMATOR_H
#define __POSE_ESTIMATOR_H

/**
 * @defgroup    drv_pose_estimator    Pose estimator
 * @ingroup     drv
 * @brief       Extended Kalman filter on [x, y, heading] from wheel odometry and LH2 fixes
 *
 * The state is the wheel-axle midpoint and the heading, in the screen frame
 * the rest of the system uses: x right, y down, heading 0 facing +y and
 * positive clockwise, so body-forward is (-sin, +cos). Predict runs on every
 * scheduler tick from encoder deltas over DB_TRACK_EFFECTIVE; update runs once
 * per new LH2 fix, with a position-only measurement of the photodiode, which
 * sits a lever arm ahead of the axle. That offset is what makes heading
 * observable while the robot turns in place.
 *
 * Process noise grows with distance travelled, never with the call rate, and
 * the heading's share also with how fast the robot turns, since slip is
 * dominated by turning. A fix whose squared Mahalanobis distance exceeds the
 * gate is rejected.
 *
 * Life cycle: SEEDING until a chain of consistent fixes has seen the photodiode
 * move far enough in the body frame to solve for heading; then TRACKING; LOST
 * once no fix has been accepted for the timeout. While LOST, predict carries on,
 * a fix inside the gate returns to TRACKING, and a new consistent chain reseeds
 * the whole pose. Heading and pose are only valid while TRACKING.
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
#define DB_POSE_ESTIMATOR_TICK_MS (10U)

/// LH2 position variance per axis, in mm^2.
/// TODO: placeholder. At rest the solve does not leave one integer mm and spin
/// fixes fit a circle to 1.5 mm rms; set from the moving LH2 noise measurement.
#define DB_POSE_ESTIMATOR_R_POS_MM2 (9.0f)

/// Position variance added per mm the axle midpoint travels, in mm^2 / mm.
/// TODO: placeholder, sized from 5 to 12 mm endpoint spread over 0.6 m.
#define DB_POSE_ESTIMATOR_Q_POS_MM2_PER_MM (0.1f)

/// Heading variance added per mm the axle midpoint travels, in deg^2 / mm.
/// TODO: placeholder, sized from 1.8 +/- 1.1 deg of drift per 0.6 m straight.
#define DB_POSE_ESTIMATOR_Q_HEADING_ROLL_DEG2_PER_MM (0.005f)

/// Heading variance added per mm of wheel travel difference |d_right - d_left|,
/// in deg^2 / mm, at or below the reference turning speed.
/// TODO: placeholder, sized for 5 deg over a 180 deg spin.
#define DB_POSE_ESTIMATOR_Q_HEADING_TURN_DEG2_PER_MM (0.1f)

/// Wheel speed difference |v_right - v_left|, in mm/s, above which the turn
/// term grows in proportion: a spin at 125 mm/s per wheel.
/// TODO: placeholder, from the effective track holding at 80 to 82 mm up to
/// about 150 mm/s per wheel and rising beyond.
#define DB_POSE_ESTIMATOR_TURN_SPEED_REF_MM_S (250.0f)

/// Squared Mahalanobis distance above which a fix is rejected: chi-square with
/// 2 degrees of freedom at 99.9 %. TODO: set from the outlier distribution.
#define DB_POSE_ESTIMATOR_GATE (13.8f)

/// Scheduler ticks without an accepted fix before TRACKING becomes LOST (1 s)
#define DB_POSE_ESTIMATOR_TIMEOUT_TICKS (100U)

/// Consistent fixes a chain needs before it may seed or reseed the pose
#define DB_POSE_ESTIMATOR_SEED_FIXES (3U)

/// Largest difference, in mm, between how far a fix lies from the chain's first
/// fix and how far odometry says the photodiode moved, for it to join the chain
#define DB_POSE_ESTIMATOR_SEED_TOLERANCE_MM (20.0f)

/// Photodiode travel in the body frame, in mm, a chain needs to solve for heading
#define DB_POSE_ESTIMATOR_ACQUIRE_MM (40.0f)

/// Life-cycle state
typedef enum {
    DB_POSE_ESTIMATOR_SEEDING,   ///< No pose; collecting a chain of consistent fixes
    DB_POSE_ESTIMATOR_TRACKING,  ///< Pose valid, fixes gated against it
    DB_POSE_ESTIMATOR_LOST,      ///< No fix accepted for the timeout; pose predicted only
} db_pose_estimator_status_t;

/// What an update did with a fix
typedef enum {
    DB_POSE_ESTIMATOR_ACCEPTED,  ///< Inside the gate, and applied
    DB_POSE_ESTIMATOR_REJECTED,  ///< Outside the gate, or broke the seed chain
    DB_POSE_ESTIMATOR_CHAINED,   ///< Joined the seed chain; no pose yet
    DB_POSE_ESTIMATOR_SEEDED,    ///< Completed a chain, and the pose was set from it
} db_pose_estimator_result_t;

/// Model and noise; the app holds one, the estimator keeps a pointer to it
typedef struct {
    float    track_mm;                    ///< track odometry divides by, mm
    float    lever_mm;                    ///< axle midpoint to photodiode, mm
    float    lever_angle_deg;             ///< direction of that offset, deg clockwise from forward
    float    r_pos_mm2;                   ///< LH2 variance per axis, mm^2
    float    q_pos_mm2_per_mm;            ///< position variance per mm travelled
    float    q_heading_roll_deg2_per_mm;  ///< heading variance per mm travelled
    float    q_heading_turn_deg2_per_mm;  ///< heading variance per mm of |d_right - d_left|
    float    turn_speed_ref_mm_s;         ///< |v_right - v_left| above which the turn term scales up, mm/s
    float    gate;                        ///< squared Mahalanobis rejection threshold
    uint32_t timeout_ticks;               ///< ticks without an accepted fix before LOST
    uint32_t seed_fixes;                  ///< fixes a seed chain needs
    float    seed_tolerance_mm;           ///< chain consistency tolerance, mm
    float    acquire_mm;                  ///< body-frame photodiode travel needed for heading, mm
} db_pose_estimator_conf_t;

/// Estimator state
typedef struct {
    const db_pose_estimator_conf_t *conf;                ///< model and noise, not owned
    db_pose_estimator_status_t      status;              ///< life-cycle state
    float                           x;                   ///< axle midpoint, mm
    float                           y;                   ///< axle midpoint, mm
    float                           theta;               ///< heading, rad, in [-pi, pi)
    float                           P[3][3];             ///< covariance of [x mm, y mm, theta rad]
    uint32_t                        ticks_since_accept;  ///< saturating
    float                           chain_x;             ///< first fix of the seed chain, mm
    float                           chain_y;             ///< first fix of the seed chain, mm
    float                           chain_bx;            ///< axle travel since that fix, mm, in the body frame at that fix
    float                           chain_by;            ///< axle travel since that fix, mm, in the body frame at that fix
    float                           chain_dtheta;        ///< rotation since that fix, rad
    float                           chain_var_theta;     ///< heading variance odometry added since that fix, rad^2
    uint32_t                        chain_count;         ///< fixes in the chain, 0 when none
    float                           last_d2;             ///< squared Mahalanobis distance of the last gated fix
    uint32_t                        predicts;            ///< predict calls, wraps
    uint32_t                        accepted;            ///< fixes applied, wraps
    uint32_t                        rejected;            ///< fixes rejected, wraps
    uint32_t                        seeds;               ///< pose seeded or reseeded from a chain, wraps
} db_pose_estimator_t;

//=========================== prototypes =======================================

/**
 * @brief   Bind the estimator to its model and start SEEDING
 *
 * @param[out]  est     Estimator state
 * @param[in]   conf    Model and noise, which must outlive the estimator
 */
void db_pose_estimator_init(db_pose_estimator_t *est, const db_pose_estimator_conf_t *conf);

/**
 * @brief   Set the pose outright and start TRACKING
 *
 * @param[in]   est             Estimator state
 * @param[in]   x_mm            Axle midpoint
 * @param[in]   y_mm            Axle midpoint
 * @param[in]   heading_deg     Heading, 0 along +y, clockwise positive
 * @param[in]   heading_sd_deg  Standard deviation of that heading
 */
void db_pose_estimator_seed(db_pose_estimator_t *est, float x_mm, float y_mm, float heading_deg, float heading_sd_deg);

/**
 * @brief   Propagate the pose over one scheduler step of wheel travel
 *
 * @param[in]   est             Estimator state
 * @param[in]   counts_left     Left encoder counts since the previous call, signed
 * @param[in]   counts_right    Right encoder counts since the previous call, signed
 * @param[in]   elapsed_ticks   Scheduler ticks since the previous call, for the fix timeout
 */
void db_pose_estimator_predict(db_pose_estimator_t *est, int32_t counts_left, int32_t counts_right, uint32_t elapsed_ticks);

/**
 * @brief   Take one LH2 fix of the photodiode
 *
 * Call once per new fix sequence: the same fix applied twice shrinks the
 * covariance without new information.
 *
 * @param[in]   est     Estimator state
 * @param[in]   x_mm    Photodiode position
 * @param[in]   y_mm    Photodiode position
 *
 * @return  what was done with the fix
 */
db_pose_estimator_result_t db_pose_estimator_update(db_pose_estimator_t *est, float x_mm, float y_mm);

/**
 * @brief   Heading, while TRACKING
 *
 * @param[in]   est     Estimator state
 * @param[out]  deg     Heading in [-180, 180), 0 along +y, clockwise positive
 *
 * @return  true while TRACKING, else false and deg untouched
 */
bool db_pose_estimator_heading_deg(const db_pose_estimator_t *est, float *deg);

/**
 * @brief   Estimated photodiode position, while TRACKING
 *
 * @param[in]   est     Estimator state
 * @param[out]  x_mm    Photodiode position
 * @param[out]  y_mm    Photodiode position
 *
 * @return  true while TRACKING, else false and the outputs untouched
 */
bool db_pose_estimator_sensor(const db_pose_estimator_t *est, float *x_mm, float *y_mm);

#endif
