/**
 * @file
 * @ingroup samples_bsp
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the motor driver in the DotBot board.
 *
 * Load this program on your board. Both wheels hold one PWM forward, brake, coast, then hold
 * the same PWM backward, brake and coast again; the outer loop sweeps that PWM.
 *
 * Holding one value per phase, rather than ramping, means every brake and every coast starts
 * from the same speed, so the two can be told apart by hand: after a brake the wheels resist
 * being turned, after a coast they spin freely. Sweeping the PWM in small steps is how the
 * breakaway value for a surface is found.
 *
 * @copyright Inria, 2022
 *
 */
#include <stdint.h>
#include <nrf.h>
// Include BSP packages
#include "board.h"
#include "motors.h"
#include "timer.h"

//=========================== defines ==========================================

#define TIMER_DEV (0)

/// PWM sweep bounds, inclusive. Equal bounds hold a single value.
#define SWEEP_PWM_START (40)
#define SWEEP_PWM_STOP  (100)
#define SWEEP_PWM_STEP  (10)

#define HOLD_MS (2000)  ///< How long each drive phase holds its PWM
#define STOP_MS (500)   ///< How long each brake and each coast lasts

//=========================== main =============================================

int main(void) {

    // Turn ON the DotBot board regulator
    db_board_init();

    // Initialize the timer
    db_timer_init(TIMER_DEV);

    // Configure Motors
    db_motors_init();

    while (1) {
        for (int16_t pwm = SWEEP_PWM_START; pwm <= SWEEP_PWM_STOP; pwm += SWEEP_PWM_STEP) {
            // Forward, at one PWM
            db_motors_set_speed(pwm, pwm);
            db_timer_delay_ms(TIMER_DEV, HOLD_MS);

            // Brake: windings shorted. The wheels should stop dead, and resist
            // being turned by hand for as long as this lasts.
            db_motors_brake();
            db_timer_delay_ms(TIMER_DEV, STOP_MS);

            // Coast: both inputs low, outputs high-impedance. Same entry speed as
            // the brake above, so the difference in stopping distance and in how
            // freely the wheels turn by hand is the whole comparison.
            db_motors_set_speed(0, 0);
            db_timer_delay_ms(TIMER_DEV, STOP_MS);

            // Backward, at the same PWM
            db_motors_set_speed(-pwm, -pwm);
            db_timer_delay_ms(TIMER_DEV, HOLD_MS);

            db_motors_brake();
            db_timer_delay_ms(TIMER_DEV, STOP_MS);
            db_motors_set_speed(0, 0);
            db_timer_delay_ms(TIMER_DEV, STOP_MS);
        }
    }
}
