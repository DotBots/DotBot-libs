#ifndef __GEOMETRY_H
#define __GEOMETRY_H

/**
 * @defgroup    drv_geometry    Robot geometry
 * @ingroup     drv
 * @brief       Physical dimensions of the robot
 *
 * Every consumer of the encoders reads its geometry from here, so a
 * measurement lands in one place and the drivers cannot drift apart.
 *
 * These values are per-board but deliberately do not live in bsp/conf/ with
 * the rest of the board configuration: control_loop.c is also compiled on the
 * host for the simulator, with only drv/ on the include path, and reaching
 * bsp/conf/ means going through board_config.h, which pulls in gpio.h and the
 * nRF headers. Keep this header free of hardware dependencies so both builds
 * can read it.
 *
 * @{
 * @file
 * @copyright Inria, 2026
 * @}
 */

#include <math.h>

#if defined(BOARD_DOTBOT_V3)
#define DB_WHEEL_DIAMETER (44.0f)  ///< Wheel diameter in mm
#define DB_TRACK          (78.0f)  ///< Distance between the two wheel mid-planes in mm
/// Quadrature counts per motor shaft revolution: a 7 PPR encoder decoded x4.
/// Bench check, wheel pushed by hand: 1400 counts per wheel revolution.
#define DB_ENCODER_CPR (28.0f)
// DotBot v1 and v2, none of whose dimensions have been measured. These are the
// values both drivers carried before the v3 bench run.
#else
#define DB_WHEEL_DIAMETER (40.0f)  ///< Wheel diameter in mm
#define DB_TRACK          (90.0f)  ///< Distance between the two wheel mid-planes in mm
#define DB_ENCODER_CPR    (12.0f)  ///< Quadrature counts per motor shaft revolution
#endif

#define DB_GEAR_RATIO (50.0f)  ///< Motor shaft revolutions per wheel revolution

/// mm of wheel travel per encoder count
#define DB_MM_PER_COUNT (((float)M_PI * DB_WHEEL_DIAMETER) / (DB_ENCODER_CPR * DB_GEAR_RATIO))

#endif
