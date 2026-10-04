/**
 * @file
 * @ingroup drv_lh2_fusion
 *
 * @brief  LH2 sweep fusion and per-station health
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "lh2_fusion.h"

//=========================== defines ==========================================

/// Weight of a new innovation in a station's running mean
#define INNOVATION_ALPHA (1.0f / 16.0f)

//=========================== public ===========================================

void db_lh2_fusion_init(db_lh2_fusion_t *fusion) {
    memset(fusion, 0, sizeof(*fusion));
    fusion->gate = DB_LH2_FUSION_GATE;
}

db_pose_estimator_result_t db_lh2_fusion_update(db_lh2_fusion_t *fusion, db_pose_estimator_t *est, const db_lh2_floor_line_t *line) {
    if (line->station >= DB_LH2_FUSION_STATIONS || est->status != DB_POSE_ESTIMATOR_TRACKING) {
        return DB_POSE_ESTIMATOR_REJECTED;
    }
    if (!isfinite(line->nx) || !isfinite(line->ny) || !isfinite(line->d_mm) || !isfinite(line->var_mm2) || !(line->var_mm2 > 0) || !(line->nx * line->nx + line->ny * line->ny > 1e-12f)) {
        return DB_POSE_ESTIMATOR_REJECTED;
    }

    db_pose_estimator_result_t result = db_pose_estimator_update_line(est, line->nx, line->ny, line->d_mm, line->var_mm2, fusion->gate);
    if (!isfinite(est->last_innovation_mm)) {
        return result;
    }

    db_lh2_station_health_t *health = &fusion->station[line->station];
    health->innovation_mean_mm += INNOVATION_ALPHA * (est->last_innovation_mm - health->innovation_mean_mm);
    if (result == DB_POSE_ESTIMATOR_ACCEPTED) {
        if (health->accepted < UINT16_MAX) {
            health->accepted++;
        }
    } else if (health->rejected < UINT16_MAX) {
        health->rejected++;
    }
    return result;
}
