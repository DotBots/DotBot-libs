#ifndef __LH2_FUSION_H
#define __LH2_FUSION_H

/**
 * @defgroup    drv_lh2_fusion    LH2 sweep fusion
 * @ingroup     drv
 * @brief       Every sweep of every station into the pose estimator, gated, with per-station health
 *
 * Each floor line is one scalar measurement of the photodiode, applied with
 * db_pose_estimator_update_line() while the estimator tracks. A station's
 * health is the running mean of its signed innovations, which drifts from
 * zero when the station was moved after calibration, and its accepted and
 * rejected counts, whose ratio rises with reflections.
 *
 * No hardware calls, so the module also builds on the host for its tests.
 *
 * @{
 * @file
 * @copyright Inria, 2026
 * @}
 */

#include <stdint.h>

#include "lh2_geometry.h"
#include "pose_estimator.h"

//=========================== defines ==========================================

/// Stations with a health record, one per lh_index
#define DB_LH2_FUSION_STATIONS (16U)

/// Squared Mahalanobis distance above which a line is rejected: chi-square,
/// 1 degree of freedom, 99.9 %
#define DB_LH2_FUSION_GATE (10.83f)

_Static_assert(DB_LH2_FUSION_STATIONS == DB_LH2_GEOMETRY_STATIONS, "one health record per station the geometry knows");

/// One station's health
typedef struct {
    float    innovation_mean_mm;  ///< exponential mean of the signed innovation of its gated lines, alpha 1/16
    uint16_t accepted;            ///< lines applied, saturating
    uint16_t rejected;            ///< lines outside the gate, saturating
} db_lh2_station_health_t;

/// Fusion state
typedef struct {
    db_lh2_station_health_t station[DB_LH2_FUSION_STATIONS];  ///< by lh_index
    float                   gate;                             ///< squared Mahalanobis threshold, DB_LH2_FUSION_GATE at init
} db_lh2_fusion_t;

//=========================== prototypes =======================================

/**
 * @brief   Clear every station's health and set the default gate
 *
 * @param[out]  fusion  Fusion state
 */
void db_lh2_fusion_init(db_lh2_fusion_t *fusion);

/**
 * @brief   Apply one floor line to the estimator and count it against its station
 *
 * Health changes only for a line the estimator gated: one that arrives while
 * it is not TRACKING, or is malformed, is REJECTED and not counted.
 *
 * @param[in,out]   fusion  Fusion state
 * @param[in,out]   est     Pose estimator
 * @param[in]       line    Floor line
 *
 * @return  what the estimator did with the line
 */
db_pose_estimator_result_t db_lh2_fusion_update(db_lh2_fusion_t *fusion, db_pose_estimator_t *est, const db_lh2_floor_line_t *line);

#endif
