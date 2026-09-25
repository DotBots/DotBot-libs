#ifndef __WHEEL_CONTROL_H
#define __WHEEL_CONTROL_H

/**
 * @defgroup    drv_wheel_control    Wheel velocity control
 * @ingroup     drv
 * @brief       Per-wheel speed loop: encoder counts in, motor duty out
 *
 * One PI controller per wheel, stepped on a fixed scheduler tick. The setpoint
 * is in mm/s; encoder counts are converted with DB_MM_PER_COUNT.
 *
 * Output = feedforward + PI. The feedforward follows the sign of the setpoint:
 * the running line u_run + k_run x |setpoint| on a turning wheel, and on a
 * standing wheel (no counts for a few steps) a kick of at least u_breakaway
 * that grows by kick_ramp per step until the wheel turns. A turning wheel more
 * than i_zone over its setpoint gets a command below u_run pushed past it.
 *
 * A zero setpoint outputs zero duty, clears the integral and the kick, and sets
 * the brake flag until the wheel has stood for as long as a standing wheel
 * takes to be detected.
 *
 * A wheel held at or above stall_pwm with no counts for stall_ms is stalled:
 * it outputs zero duty without braking (the motor coasts) until the setpoint
 * changes or the loop is reset.
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
#define DB_WHEEL_CONTROL_TICK_MS (10U)

/// Distance a wheel runs on after its setpoint drops to zero, per mm/s of speed,
/// in seconds: braking to zero stops the v3 wheel in about 0.05 s times the speed
#define DB_WHEEL_GOAL_RUN_ON_S (0.05f)

/// Gains and limits, shared by both wheels or given per wheel
typedef struct {
    float    kp;                 ///< duty per mm/s of speed error
    float    ki;                 ///< duty per mm of accumulated speed error
    float    u_breakaway;        ///< least duty applied to a standing wheel
    float    kick_ramp;          ///< duty added per step while a wheel stays standing
    float    u_run;              ///< duty of the running line at zero speed
    float    k_run;              ///< slope of the running line, duty per mm/s
    float    i_zone;             ///< the integral only accumulates while |error| is below this, mm/s
    float    pwm_max;            ///< output saturation, duty
    float    pwm_slew_per_tick;  ///< largest output change in one step, duty
    float    stall_pwm;          ///< |duty| at or above which a wheel without counts is being forced, duty
    uint32_t stall_ms;           ///< time forced without a count before the wheel is stalled, ms; 0 disables
} db_wheel_control_conf_t;

/// State of one wheel's loop; the app holds one per wheel
typedef struct {
    const db_wheel_control_conf_t *conf;         ///< gains, not owned
    float                          setpoint;     ///< mm/s
    float                          integral;     ///< mm, bounded by anti-windup
    float                          pwm;          ///< last output, duty
    float                          measured;     ///< mm/s at the last step, for telemetry
    float                          previous;     ///< mm/s at the step before
    float                          ff;           ///< feedforward at the last step, duty, for telemetry
    uint32_t                       still_ticks;  ///< consecutive steps without a count, saturating
    float                          kick_boost;   ///< duty the kick ramp has added so far
    bool                           brake;        ///< short the motor instead of applying the duty
    uint32_t                       forced_ms;    ///< time at or above stall_pwm without a count, ms
    bool                           stalled;      ///< coasting after a stall, until the setpoint changes
} db_wheel_control_t;

/// Body motion, the input of the twist mixer
typedef struct {
    float v_mm_s;       ///< forward speed, positive forward
    float omega_deg_s;  ///< turn rate, positive clockwise (y down, heading 0 along +y)
} db_body_twist_t;

/// A distance per wheel, driven on the speed loop until the encoders say it is done
typedef struct {
    float target_left_mm;      ///< signed distance for the left wheel
    float target_right_mm;     ///< signed distance for the right wheel
    float travelled_left_mm;   ///< signed distance counted since the start
    float travelled_right_mm;  ///< signed distance counted since the start
    float speed_mm_s;          ///< speed of the wheel with the longer distance
    bool  active;              ///< driving; false once arrived or never started
} db_wheel_goal_t;

//=========================== prototypes =======================================

/**
 * @brief   Bind a wheel to its gains and zero its state
 *
 * @param[out]  wheel   Wheel state
 * @param[in]   conf    Gains, which must outlive the wheel
 */
void db_wheel_control_init(db_wheel_control_t *wheel, const db_wheel_control_conf_t *conf);

/**
 * @brief   Set the target speed
 *
 * A change of sign clears the integral; any change of value clears a stall.
 *
 * @param[in]   wheel       Wheel state
 * @param[in]   mm_per_s    Target speed, positive forward
 */
void db_wheel_control_set_setpoint(db_wheel_control_t *wheel, float mm_per_s);

/**
 * @brief   Zero the setpoint, the integral and the output, and clear a stall
 *
 * @param[in]   wheel   Wheel state
 */
void db_wheel_control_reset(db_wheel_control_t *wheel);

/**
 * @brief   Run one loop step
 *
 * @param[in]   wheel           Wheel state
 * @param[in]   delta_counts    Encoder counts since the previous step, signed
 * @param[in]   elapsed_ticks   Ticks since the previous step, normally 1; 0 returns the previous output
 *
 * @return  motor duty in [-pwm_max, pwm_max], ignored while wheel->brake is set
 */
int8_t db_wheel_control_step(db_wheel_control_t *wheel, int32_t delta_counts, uint32_t elapsed_ticks);

/**
 * @brief   Combine a QDEC read with its double transitions
 *
 * Each double transition is two steps in the direction the accumulator moved.
 * A wheel cannot reverse inside a window fast enough to produce doubles, so
 * that direction is the wheel's. With no single steps to give a direction the
 * doubles are dropped.
 *
 * @param[in]   acc     Signed accumulator
 * @param[in]   dbl     Double transitions in the same window
 *
 * @return  signed counts
 */
int32_t db_wheel_control_counts(int32_t acc, uint32_t dbl);

/**
 * @brief   Wheel speeds for a body twist, over the effective track for its turn radius
 *
 * Clockwise turns speed the left wheel up: v_left = v + w L/2, v_right = v - w L/2.
 *
 * @param[in]   twist       Body motion
 * @param[out]  left_mm_s   Left wheel speed
 * @param[out]  right_mm_s  Right wheel speed
 */
void db_wheel_control_from_twist(const db_body_twist_t *twist, float *left_mm_s, float *right_mm_s);

/**
 * @brief   Start an odometric goal: a signed distance per wheel
 *
 * The wheel with the longer distance runs at the given speed and the other one
 * in proportion, so both finish together. Nothing is driven if both distances
 * are zero or the speed is not positive.
 *
 * @param[out]  goal        Goal state
 * @param[in]   left_mm     Left wheel distance, positive forward
 * @param[in]   right_mm    Right wheel distance, positive forward
 * @param[in]   speed_mm_s  Speed of the wheel with the longer distance, positive
 */
void db_wheel_goal_start(db_wheel_goal_t *goal, float left_mm, float right_mm, float speed_mm_s);

/**
 * @brief   Start a straight move, backward for a negative distance
 *
 * @param[out]  goal        Goal state
 * @param[in]   distance_mm Distance, positive forward
 * @param[in]   speed_mm_s  Wheel speed, positive
 */
void db_wheel_goal_straight(db_wheel_goal_t *goal, float distance_mm, float speed_mm_s);

/**
 * @brief   Start a turn in place over DB_TRACK_EFFECTIVE
 *
 * @param[out]  goal        Goal state
 * @param[in]   angle_deg   Rotation, positive clockwise
 * @param[in]   speed_mm_s  Wheel speed, positive
 */
void db_wheel_goal_turn(db_wheel_goal_t *goal, float angle_deg, float speed_mm_s);

/**
 * @brief   Count one step of encoder travel and give the wheel setpoints
 *
 * The goal arrives once the longer wheel is within its braking run-on
 * (DB_WHEEL_GOAL_RUN_ON_S times the speed) of its distance; from then on both
 * setpoints are zero, which the speed loop turns into braking to a stand.
 *
 * @param[in]   goal            Goal state
 * @param[in]   delta_left      Left encoder counts since the previous step, signed
 * @param[in]   delta_right     Right encoder counts since the previous step, signed
 * @param[out]  left_mm_s       Left wheel setpoint
 * @param[out]  right_mm_s      Right wheel setpoint
 *
 * @return  true while the goal is still driving
 */
bool db_wheel_goal_step(db_wheel_goal_t *goal, int32_t delta_left, int32_t delta_right, float *left_mm_s, float *right_mm_s);

#endif
