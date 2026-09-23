#ifndef __WHEEL_CONTROL_H
#define __WHEEL_CONTROL_H

/**
 * @defgroup    drv_wheel_control    Wheel velocity control
 * @ingroup     drv
 * @brief       Per-wheel speed loop: encoder counts in, motor duty out
 *
 * One PI controller per wheel, stepped on a fixed scheduler tick. The setpoint
 * is in mm/s; the only unit change is the encoder measurement inside
 * db_wheel_control_step(), counts to mm/s with DB_MM_PER_COUNT.
 *
 * The output is the PI plus a feedforward that follows the sign of the
 * setpoint and is zero at a zero setpoint: the running line
 * (u_run + k_run x |setpoint|) once the wheel turns, and while it is stalled
 * (no counts for a few steps) a kick of at least u_breakaway that ramps up
 * until the wheel moves, since the duty that frees a wheel changes with where
 * it came to rest. The PI trims around that
 * feedforward. On a turning wheel more than i_zone over its setpoint, a
 * command that comes out below u_run is moved past it, since there the motor
 * does not drive and the wheel would only coast. A zero setpoint outputs zero
 * duty and clears the integral and the kick, so a stopped wheel never creeps,
 * and sets the wheel's brake flag while the wheel still turns: the app shorts
 * that motor until the wheel has gone a stall's worth of steps without a
 * count, then lets it coast.
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

/// Gains and limits, shared by both wheels or given per wheel
typedef struct {
    float kp;                 ///< duty per mm/s of speed error
    float ki;                 ///< duty per mm of accumulated speed error
    float u_breakaway;        ///< least duty applied to a stalled wheel
    float kick_ramp;          ///< duty added per step while a wheel stays stalled
    float u_run;              ///< duty of the running line at zero speed
    float k_run;              ///< slope of the running line, duty per mm/s
    float i_zone;             ///< the integral only accumulates while |error| is below this, mm/s
    float pwm_max;            ///< output saturation, duty
    float pwm_slew_per_tick;  ///< largest output change in one step, duty
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
    float                          kick_boost;   ///< duty the stall ramp has added so far
    bool                           brake;        ///< short the motor instead of applying the duty
} db_wheel_control_t;

/// Body motion, the input of the twist mixer
typedef struct {
    float v_mm_s;       ///< forward speed, positive forward
    float omega_deg_s;  ///< turn rate, positive clockwise (y down, heading 0 along +y)
} db_body_twist_t;

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
 * A change of sign clears the integral.
 *
 * @param[in]   wheel       Wheel state
 * @param[in]   mm_per_s    Target speed, positive forward
 */
void db_wheel_control_set_setpoint(db_wheel_control_t *wheel, float mm_per_s);

/**
 * @brief   Zero the setpoint, the integral and the output
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
 * @brief   Wheel speeds for a body twist, over the track DB_TRACK
 *
 * Clockwise turns speed the left wheel up: v_left = v + w L/2, v_right = v - w L/2.
 *
 * @param[in]   twist       Body motion
 * @param[out]  left_mm_s   Left wheel speed
 * @param[out]  right_mm_s  Right wheel speed
 */
void db_wheel_control_from_twist(const db_body_twist_t *twist, float *left_mm_s, float *right_mm_s);

#endif
