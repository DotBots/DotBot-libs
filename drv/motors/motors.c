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
#include <stdbool.h>
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

//=========================== private ==========================================

/// Fill the two channels of one bridge for a duty, or for the brake
static void _bridge_channels(uint16_t *channels, int16_t pwm, bool brake) {
    if (brake) {
        // Full duty on both inputs holds them high, which is the DRV8833
        // brake state. db_pwm_channels_set sets bit 15 on every value, so the
        // compare is the high time and M_TOP is the whole period.
        channels[0] = M_TOP;
        channels[1] = M_TOP;
        return;
    }
    if (pwm > 100) {
        pwm = 100;
    }
    if (pwm < -100) {
        pwm = -100;
    }
    // Positive values turn the motor forward, negative backward
    channels[0] = (pwm >= 0) ? (uint16_t)pwm : 0;
    channels[1] = (pwm >= 0) ? 0 : (uint16_t)(-pwm);
}

//=========================== public ==========================================

void db_motors_init(void) {
    db_pwm_init(PWM_DEV, db_motors_pins, PWM_CHANNELS, M_TOP);
}

void db_motors_set_pwm_brake(int16_t l_pwm, int16_t r_pwm, bool l_brake, bool r_brake) {
    uint16_t pwm_seq[PWM_CHANNELS];

    _bridge_channels(&pwm_seq[0], l_pwm, l_brake);
    _bridge_channels(&pwm_seq[2], r_pwm, r_brake);
    db_pwm_channels_set(PWM_DEV, pwm_seq);
}

void db_motors_set_pwm(int16_t l_pwm, int16_t r_pwm) {
    db_motors_set_pwm_brake(l_pwm, r_pwm, false, false);
}

void db_motors_brake(void) {
    db_motors_set_pwm_brake(0, 0, true, true);
}

void db_motors_coast(void) {
    db_motors_set_pwm_brake(0, 0, false, false);
}
