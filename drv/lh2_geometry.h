#ifndef __LH2_GEOMETRY_H
#define __LH2_GEOMETRY_H

/**
 * @defgroup    drv_lh2_geometry    LH2 sweep geometry
 * @ingroup     drv
 * @brief       From a station's two LFSR counts to a camera point and to floor lines
 *
 * A station is a pinhole camera whose image point is computed from the two
 * sweep angles. Each sweep alone is a plane through the station, which the
 * image plane cuts in a line and a homography maps to a line on the floor; the
 * two lines of one pair meet at the homography of the camera point.
 *
 * No hardware calls, so the module also builds on the host for its tests.
 *
 * @{
 * @file
 * @copyright Inria, 2026
 * @}
 */

#include <stddef.h>
#include <stdint.h>

//=========================== defines ==========================================

/// Stations the period table covers, one per LH2 channel, indexed by lh_index
#define DB_LH2_GEOMETRY_STATIONS (16U)

/// LFSR counts per rotor revolution are the station's period divided by this
#define DB_LH2_PERIOD_TICKS_PER_COUNT (8U)

/// Standard deviation of one sweep angle, rad, which sets a floor line's variance.
/// TODO: placeholder, 2 mm at 2 m; the two-station bench run measures it.
#define DB_LH2_SWEEP_SIGMA_RAD (1e-3f)

/// One sweep as a line on the floor. Naturally aligned: the secure side stores
/// through a veneer with the unaligned-access trap enabled.
typedef struct {
    float   nx;       ///< unit normal of the floor line, frame axes, pointing the way the line moves as the sweep angle grows
    float   ny;       ///< unit normal of the floor line, frame axes
    float   d_mm;     ///< the photodiode satisfies nx * x + ny * y = d_mm
    float   var_mm2;  ///< variance along the normal, from DB_LH2_SWEEP_SIGMA_RAD through the homography
    uint8_t station;  ///< lh_index 0..15
    uint8_t sweep;    ///< 0: the line of count1, 1: the line of count2
    uint8_t _pad[2];
} db_lh2_floor_line_t;

_Static_assert(sizeof(db_lh2_floor_line_t) == 20, "db_lh2_floor_line_t is part of the NSC ABI");
_Static_assert(offsetof(db_lh2_floor_line_t, nx) == 0, "db_lh2_floor_line_t is part of the NSC ABI");
_Static_assert(offsetof(db_lh2_floor_line_t, ny) == 4, "db_lh2_floor_line_t is part of the NSC ABI");
_Static_assert(offsetof(db_lh2_floor_line_t, d_mm) == 8, "db_lh2_floor_line_t is part of the NSC ABI");
_Static_assert(offsetof(db_lh2_floor_line_t, var_mm2) == 12, "db_lh2_floor_line_t is part of the NSC ABI");
_Static_assert(offsetof(db_lh2_floor_line_t, station) == 16, "db_lh2_floor_line_t is part of the NSC ABI");
_Static_assert(offsetof(db_lh2_floor_line_t, sweep) == 17, "db_lh2_floor_line_t is part of the NSC ABI");

//=========================== prototypes =======================================

/**
 * @brief   Rotor period of a station, in the units of the LFSR count times DB_LH2_PERIOD_TICKS_PER_COUNT
 *
 * @param[in]   station     lh_index; 0 when out of range
 */
uint32_t db_lh2_period(uint8_t station);

/**
 * @brief   Pinhole image point of one sweep pair
 *
 * @param[in]   count1      sweep 0 LFSR count
 * @param[in]   count2      sweep 1 LFSR count
 * @param[in]   station     lh_index; NaN out when out of range
 * @param[out]  cam         image point (x, y)
 */
void db_lh2_camera_point(uint32_t count1, uint32_t count2, uint8_t station, double cam[2]);

/**
 * @brief   Both sweeps of one pair as floor lines
 *
 * @param[in]   count1      sweep 0 LFSR count
 * @param[in]   count2      sweep 1 LFSR count
 * @param[in]   station     lh_index
 * @param[in]   homography  camera point to frame mm
 * @param[out]  out         out[0] is the count1 sweep, out[1] the count2 one
 *
 * @return  2, or 0 when the station is out of range, the homography singular or a line degenerate
 */
uint8_t db_lh2_sweep_lines(uint32_t count1, uint32_t count2, uint8_t station, const float homography[3][3], db_lh2_floor_line_t out[2]);

#endif
