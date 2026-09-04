/**
 * @file
 * @ingroup drv_motors
 *
 * @brief  nRF52833-specific definition of the "motors bsp module.
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <stdint.h>
#include <nrf.h>

#include "board_config.h"
#include "motors.h"
#include "pwm.h"

//=========================== defines ==========================================

// Max value of the PWM counter register (100us => 10kHz)
#define M_TOP (100)
// 4 PWM channels are used
#define PWM_DEV      (0)
#define PWM_CHANNELS (4)

//=========================== public ==========================================

void db_motors_init(void) {
    db_pwm_init(PWM_DEV, db_motors_pins, PWM_CHANNELS, M_TOP);
}

void db_motors_brake(void) {
    // Full duty on both inputs of each bridge holds them high, which is the
    // DRV8833 brake state. db_pwm_channels_set sets bit 15 on every value, so
    // the compare is the high time and M_TOP is the whole period.
    uint16_t pwm_seq[PWM_CHANNELS] = { M_TOP, M_TOP, M_TOP, M_TOP };

    db_pwm_channels_set(PWM_DEV, pwm_seq);
}

void db_motors_set_speed(int16_t l_speed, int16_t r_speed) {

    // Double check for out-of-bound values.
    if (l_speed > 100)
        l_speed = 100;
    if (r_speed > 100)
        r_speed = 100;

    if (l_speed < -100)
        l_speed = -100;
    if (r_speed < -100)
        r_speed = -100;

    uint16_t pwm_seq[PWM_CHANNELS] = { 0 };

    // Left motor processing
    if (l_speed >= 0)  // Positive values turn the motor forward.
    {
        pwm_seq[0] = l_speed;
        pwm_seq[1] = 0;
    }
    if (l_speed < 0)  // Negative values turn the motor backward.
    {
        l_speed *= -1;  // remove the negative before loading into memory

        pwm_seq[0] = 0;
        pwm_seq[1] = l_speed;
    }

    // Right motor processing
    if (r_speed >= 0)  // Positive values turn the motor forward.
    {
        pwm_seq[2] = r_speed;
        pwm_seq[3] = 0;
    }
    if (r_speed < 0)  // Negative values turn the motor backward.
    {
        r_speed *= -1;  // remove the negative before loading into memory

        pwm_seq[2] = 0;
        pwm_seq[3] = r_speed;
    }

    // Update PWM values
    db_pwm_channels_set(PWM_DEV, pwm_seq);
}
