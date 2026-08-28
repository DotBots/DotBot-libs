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
 * @{
 * @file
 * @copyright Inria, 2026
 * @}
 */

#include <math.h>

#if defined(BOARD_DOTBOT_V3)
#define DB_WHEEL_DIAMETER (44.0f)  ///< Wheel diameter in mm
#define DB_TRACK          (78.0f)  ///< Distance between the two wheel mid-planes in mm
// DotBot v1 and v2, neither of which has been measured
#else
#define DB_WHEEL_DIAMETER (40.0f)  ///< Wheel diameter in mm
#define DB_TRACK          (90.0f)  ///< Distance between the two wheel mid-planes in mm
#endif

#define DB_GEAR_RATIO (50.0f)  ///< Motor shaft revolutions per wheel revolution

/// Quadrature counts per motor shaft revolution. A hand-turn bench reading
/// disagrees with this by a factor of about 24, so treat anything derived from
/// encoder distance as provisional until a driven measurement settles it.
#define DB_ENCODER_CPR (12.0f)

/// mm of wheel travel per encoder count
#define DB_MM_PER_COUNT (((float)M_PI * DB_WHEEL_DIAMETER) / (DB_ENCODER_CPR * DB_GEAR_RATIO))

#endif
