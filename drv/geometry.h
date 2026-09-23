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
 * the rest of the board configuration: %control_loop.c is also compiled on the
 * host for the simulator, with only drv/ on the include path, and reaching
 * bsp/conf/ means going through %board_config.h, which pulls in %gpio.h and the
 * nRF headers. Keep this header free of hardware dependencies so both builds
 * can read it.
 *
 * Units across drv/: lengths in millimetres, speeds in mm/s, angles in
 * degrees. Radians and metres appear only inside a function that converts.
 *
 * @{
 * @file
 * @copyright Inria, 2026
 * @}
 */

#include <math.h>

#if defined(BOARD_DOTBOT_V3)
/// Wheel diameter in mm, caliper-measured unloaded on a v3 (2026-09-23)
#define DB_WHEEL_DIAMETER (43.0f)

/// Distance between the two wheel mid-planes in mm. The caliper reads 77; 78 is
/// carried here so the C and Python models of the robot agree exactly.
#define DB_TRACK (78.0f)

/// Nominal quadrature counts per motor shaft revolution: 7 pulses decoded x4
#define DB_ENCODER_CPR (28.0f)

/// Hand-counted, within 0.14% of the measured 1430 ± 1.5 counts per wheel turn; sold as 50:1
#define DB_GEAR_RATIO (51.0f)

/// Distance in mm from the wheel-axle midpoint, which is where the robot turns
/// about, to the lighthouse photodiode. Rotating the robot therefore moves the
/// reported position even when the robot goes nowhere, and any estimator whose
/// state is the axle midpoint has to model that.
///
/// Board coordinates put the axle line at y = 124.5 and the photodiode at
/// (75.0, 71.0), on the centreline. Assumes a freely rolling front caster: if
/// it binds the robot pivots about it instead, and the effective offset moves
/// toward -11 mm, changing sign.
#define DB_LH2_LEVER_ARM (53.5f)

/// Direction of that offset in degrees, clockwise from body-forward. Zero
/// because the photodiode sits on the centreline.
#define DB_LH2_LEVER_ANGLE (0.0f)

/// Effective track in mm for a spin in place: the rotation the robot actually
/// makes on carpet for a given wheel travel difference, fitted against LH2.
/// Good to 1 per cent up to 120 mm/s per wheel; it reads 83 at 200 and 87 at 300.
#define DB_TRACK_EFFECTIVE (81.0f)

/// Effective track in mm for arcs of 100 mm radius and wider (fit 84.9, sd 1.5)
#define DB_TRACK_EFFECTIVE_ARC (85.0f)

/// |v_left + v_right| / |v_right - v_left|, the turn radius over half the
/// track, at which the effective track reaches DB_TRACK_EFFECTIVE_ARC: 100 mm
#define DB_TRACK_EFFECTIVE_ARC_RATIO (2.35f)

/// Lever arm in mm fitted to the LH2 fixes of spins in place: 52.0 mm
/// clockwise and 51.3 mm counter-clockwise, 52 +/- 2 with the LH2 scale
/// error. The estimator uses this one; DB_LH2_LEVER_ARM stays the board
/// figure, which is 2 mm longer.
#define DB_LH2_LEVER_ARM_EFFECTIVE (51.5f)

// DotBot v1 and v2, none of whose dimensions have been measured. These are the
// values both drivers carried before the v3 bench run, kept so those boards
// behave exactly as they did. A zero lever arm is what the drivers assumed:
// the sensor at the point of rotation. Do not "correct" these to the v3
// numbers; they describe different hardware.
#else
#define DB_WHEEL_DIAMETER            (40.0f)           ///< Wheel diameter in mm
#define DB_TRACK                     (90.0f)           ///< Distance between the two wheel mid-planes in mm
#define DB_ENCODER_CPR               (12.0f)           ///< Quadrature counts per motor shaft revolution
#define DB_GEAR_RATIO                (50.0f)           ///< Motor shaft revolutions per wheel revolution
#define DB_LH2_LEVER_ARM             (0.0f)            ///< Axle midpoint to photodiode, in mm
#define DB_LH2_LEVER_ANGLE           (0.0f)            ///< Direction of that offset, degrees clockwise from forward
#define DB_TRACK_EFFECTIVE           DB_TRACK          ///< Track odometry divides by, in mm
#define DB_TRACK_EFFECTIVE_ARC       DB_TRACK          ///< The same for wide arcs, in mm
#define DB_TRACK_EFFECTIVE_ARC_RATIO (1.0f)            ///< Unused while the two tracks are equal
#define DB_LH2_LEVER_ARM_EFFECTIVE   DB_LH2_LEVER_ARM  ///< Lever arm the estimator uses, in mm
#endif

/// mm of wheel travel per encoder count
#define DB_MM_PER_COUNT (((float)M_PI * DB_WHEEL_DIAMETER) / (DB_ENCODER_CPR * DB_GEAR_RATIO))

/**
 * @brief   Effective track for a turn, in mm
 *
 * DB_TRACK_EFFECTIVE in place, rising linearly with |left + right| /
 * |right - left| to DB_TRACK_EFFECTIVE_ARC at DB_TRACK_EFFECTIVE_ARC_RATIO and
 * held there. Only the ratio counts, so left and right may be speeds or
 * distances. Spin speed is not modelled.
 *
 * @param[in]   left    Left wheel speed or travel
 * @param[in]   right   Right wheel speed or travel
 *
 * @return  track in mm
 */
static inline float db_track_effective_mm(float left, float right) {
    float diff = fabsf(right - left);
    float sum  = fabsf(right + left);
    if (sum >= DB_TRACK_EFFECTIVE_ARC_RATIO * diff) {
        return DB_TRACK_EFFECTIVE_ARC;
    }
    return DB_TRACK_EFFECTIVE + (DB_TRACK_EFFECTIVE_ARC - DB_TRACK_EFFECTIVE) * sum / (DB_TRACK_EFFECTIVE_ARC_RATIO * diff);
}

#endif
