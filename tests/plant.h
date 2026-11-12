#ifndef __PLANT_H
#define __PLANT_H

/**
 * @file
 * @brief  A duty-driven DotBot plant for the host tests
 *
 * It inverts the wheel loop's running line with a 50 ms lag and a breakaway,
 * brakes to a stop with the same lag, emits whole encoder counts, and is
 * seen by LH2 at 10 Hz with 2 ticks of age and a set noise.
 *
 * @copyright Inria, 2026
 */

#include <stdbool.h>
#include <stdint.h>

#include "dotbot_control.h"

#define PLANT_TAU_S     (0.05f)
#define PLANT_DT_S      (0.01f)
#define PLANT_U_RUN     (32.0f)
#define PLANT_K_RUN     (0.097f)
#define PLANT_U_BREAK   (40.0f)
#define PLANT_FIX_AGE   (2U)
#define PLANT_FIX_EVERY (10U)

typedef struct {
    float    x;            ///< truth, axle midpoint, mm
    float    y;            ///< truth, axle midpoint, mm
    float    heading_deg;  ///< truth, 0 facing +y, clockwise positive
    float    v_left;       ///< mm/s
    float    v_right;      ///< mm/s
    float    frac_left;    ///< counts not yet emitted
    float    frac_right;   ///< counts not yet emitted
    int8_t   pwm_left;
    int8_t   pwm_right;
    bool     brake_left;
    bool     brake_right;
    float    noise_mm;
    uint32_t rng;
    uint32_t tick;
    float    px[PLANT_FIX_AGE + 1];  ///< photodiode history, newest first
    float    py[PLANT_FIX_AGE + 1];
    uint32_t fix_sequence;
    uint32_t fix_x;
    uint32_t fix_y;
    bool     airborne;  ///< held off the floor: the wheels turn, the robot and its fixes stay put
    float    air_x;     ///< photodiode fix while airborne, mm
    float    air_y;     ///< photodiode fix while airborne, mm
} plant_t;

/// Uniform in (0, 1), from the plant's 32-bit LCG
float plant_uniform(plant_t *p);

/// Standard normal, from the plant's LCG
float plant_gauss(plant_t *p);

/// Unit forward vector of a heading, 0 facing +y, clockwise positive
void plant_forward(float heading_deg, float *fx, float *fy);

/// At rest at (x, y) and heading_deg; noise_mm on each fix, seed for its LCG
void plant_init(plant_t *p, float x, float y, float heading_deg, float noise_mm, uint32_t seed);

/// One 10 ms tick: the wheels, the pose, the encoder counts and, every PLANT_FIX_EVERY ticks, a fix
void plant_step(plant_t *p, db_control_input_t *in);

/// The motor write a tick produced, if any
void plant_apply(plant_t *p, const db_control_output_t *out);

#endif
