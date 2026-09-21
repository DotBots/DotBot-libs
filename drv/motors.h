#ifndef __MOTORS_H
#define __MOTORS_H

/**
 * @defgroup    drv_motors  Motors driver
 * @ingroup     drv
 * @brief       Control the DC motors
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>
#include <nrf.h>

/**
 * @brief Configures the PMW0 peripheral to work with the onboard DotBot RGB Motor driver
 *
 * The DotBot uses a DRV8833 dual H-bridge driver with a 4 pmw control interface.
 * the PWM0 peripheral is used to generate the pwm signals it requires.
 *
 * PWM frequency = 10Khz
 * PWM resolution = 100 units (1us resolution)
 *
 */
void db_motors_init(void);

/**
 * @brief Set the duty cycle of the left and right motors
 *
 *  Each value is the H-bridge duty in percent of the 10 kHz PWM period, from
 *  -100 to 100, clamped to that range. It is not a speed: the wheel speed a
 *  duty produces depends on load, surface and battery. Positive drives the
 *  motor forward, negative backward. Zero coasts, as db_motors_coast() does.
 *
 * @param[in] l_pwm duty of the left motor [-100, 100]
 * @param[in] r_pwm duty of the right motor [-100, 100]
 */
void db_motors_set_pwm(int16_t l_pwm, int16_t r_pwm);

/**
 * @brief Let both motors freewheel
 *
 *  Drives both inputs of each H-bridge low, so the outputs go high-impedance
 *  and the wheels turn freely.
 */
void db_motors_coast(void);

/**
 * @brief Short the motor windings so the wheels resist being turned
 *
 *  Drives both inputs of each H-bridge high, which is the DRV8833's brake
 *  state, as opposed to db_motors_coast().
 *
 *  Released by any subsequent db_motors_set_pwm() call.
 */
void db_motors_brake(void);

#endif
