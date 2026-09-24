/**
 * @file
 * @ingroup samples_drv
 * @brief Drive a square on the wheel speed loop, forward then backward
 *
 * Each side and each corner is an odometric goal: the wheels run on the speed
 * loop until the encoders say the distance is done, then brake to a stand.
 *
 * @copyright Inria, 2026
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <nrf.h>
#include "board.h"
#include "board_config.h"
#include "motors.h"
#include "qdec.h"
#include "timer.h"
#include "wheel_control.h"

//=========================== defines ==========================================

#define TIMER_DEV     (0)
#define QDEC_LEFT     (0)
#define QDEC_RIGHT    (1)
#define SIDE_MM       (500.0f)
#define STRAIGHT_MM_S (150.0f)
#define TURN_MM_S     (100.0f)
#define PAUSE_TICKS   (30U)  ///< 300 ms standing between two moves

typedef struct {
    float distance_mm;  ///< straight move, used when angle_deg is zero
    float angle_deg;    ///< turn in place, positive clockwise
} move_t;

//=========================== variables ========================================

static const qdec_conf_t _qdec_left = {
    .pin_a = &db_qdec_left_a_pin,
    .pin_b = &db_qdec_left_b_pin,
};

static const qdec_conf_t _qdec_right = {
    .pin_a = &db_qdec_right_a_pin,
    .pin_b = &db_qdec_right_b_pin,
};

/// The v3 gains, as in the DotBot app
static const db_wheel_control_conf_t _wheel_conf = {
    .kp                = 0.52f,
    .ki                = 5.2f,
    .u_breakaway       = 44.0f,
    .kick_ramp         = 0.5f,
    .u_run             = 32.0f,
    .k_run             = 0.097f,
    .i_zone            = 38.0f,
    .pwm_max           = 100.0f,
    .pwm_slew_per_tick = 100.0f,
    .stall_pwm         = 80.0f,
    .stall_ms          = 500U,
};

static const move_t _moves[] = {
    { SIDE_MM, 0 },
    { 0, 90 },
    { SIDE_MM, 0 },
    { 0, 90 },
    { SIDE_MM, 0 },
    { 0, 90 },
    { SIDE_MM, 0 },
    { -SIDE_MM, 0 },
    { 0, -90 },
    { -SIDE_MM, 0 },
    { 0, -90 },
    { -SIDE_MM, 0 },
    { 0, -90 },
    { -SIDE_MM, 0 },
};

static db_wheel_control_t _wheel_left;
static db_wheel_control_t _wheel_right;
static db_wheel_goal_t    _goal;
static volatile uint32_t  _tick_count = 0;

//=========================== callbacks ========================================

static void _tick(void) {
    _tick_count++;
}

//=========================== private ==========================================

static void _move_start(const move_t *move) {
    if (move->angle_deg != 0) {
        db_wheel_goal_turn(&_goal, move->angle_deg, TURN_MM_S);
    } else {
        db_wheel_goal_straight(&_goal, move->distance_mm, STRAIGHT_MM_S);
    }
}

/// One tick of the speed loop; true once the goal is done and both wheels stand
static bool _move_step(void) {
    uint32_t dbl_left;
    uint32_t dbl_right;
    int32_t  left  = db_wheel_control_counts(db_qdec_read_and_clear_dbl(QDEC_LEFT, &dbl_left), dbl_left);
    int32_t  right = db_wheel_control_counts(db_qdec_read_and_clear_dbl(QDEC_RIGHT, &dbl_right), dbl_right);
    float    setpoint_left;
    float    setpoint_right;
    bool     driving = db_wheel_goal_step(&_goal, left, right, &setpoint_left, &setpoint_right);
    db_wheel_control_set_setpoint(&_wheel_left, setpoint_left);
    db_wheel_control_set_setpoint(&_wheel_right, setpoint_right);
    int8_t pwm_left  = db_wheel_control_step(&_wheel_left, left, 1);
    int8_t pwm_right = db_wheel_control_step(&_wheel_right, right, 1);
    db_motors_set_pwm_brake(pwm_left, pwm_right, _wheel_left.brake, _wheel_right.brake);
    return !driving && !_wheel_left.brake && !_wheel_right.brake;
}

//=========================== main =============================================

int main(void) {
    db_board_init();
    db_motors_init();
    db_qdec_init(QDEC_LEFT, &_qdec_left, NULL, NULL);
    db_qdec_init(QDEC_RIGHT, &_qdec_right, NULL, NULL);
    db_wheel_control_init(&_wheel_left, &_wheel_conf);
    db_wheel_control_init(&_wheel_right, &_wheel_conf);
    db_timer_init(TIMER_DEV);
    db_timer_set_periodic_ms(TIMER_DEV, 0, DB_WHEEL_CONTROL_TICK_MS, &_tick);

    uint32_t next     = 0;
    uint32_t serviced = 0;
    uint32_t standing = 0;
    _move_start(&_moves[next]);

    while (1) {
        __WFE();
        while (serviced != _tick_count) {
            serviced++;
            if (!_move_step()) {
                standing = 0;
                continue;
            }
            if (++standing < PAUSE_TICKS || next + 1 >= sizeof(_moves) / sizeof(_moves[0])) {
                continue;
            }
            standing = 0;
            _move_start(&_moves[++next]);
        }
    }
}
