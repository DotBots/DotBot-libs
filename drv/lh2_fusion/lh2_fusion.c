/**
 * @file
 * @ingroup drv_lh2_fusion
 *
 * @brief  LH2 sweep fusion and per-station health
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdbool.h>
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

bool db_lh2_fusion_intersect(const db_lh2_floor_line_t *lines, uint8_t count, float *x_mm, float *y_mm) {
    // Normal equations A p = b of sum w (n . p - d)^2, with unit normals
    float a00 = 0, a01 = 0, a11 = 0, b0 = 0, b1 = 0;
    for (uint8_t i = 0; i < count; i++) {
        const db_lh2_floor_line_t *l  = &lines[i];
        float                      n2 = l->nx * l->nx + l->ny * l->ny;
        if (!isfinite(n2) || !isfinite(l->d_mm) || !isfinite(l->var_mm2) || !(l->var_mm2 > 0) || !(n2 > 1e-12f)) {
            continue;
        }
        float n  = sqrtf(n2);
        float nx = l->nx / n, ny = l->ny / n, d = l->d_mm / n;
        float w = n2 / l->var_mm2;
        a00 += w * nx * nx;
        a01 += w * nx * ny;
        a11 += w * ny * ny;
        b0 += w * nx * d;
        b1 += w * ny * d;
    }
    float det  = a00 * a11 - a01 * a01;
    float half = 0.5f * (a00 + a11);
    if (!(half > 0) || !(det >= DB_LH2_FUSION_CROSS_MIN_SIN2 * half * half)) {
        return false;
    }
    float x = (a11 * b0 - a01 * b1) / det;
    float y = (a00 * b1 - a01 * b0) / det;
    if (!isfinite(x) || !isfinite(y)) {
        return false;
    }
    *x_mm = x;
    *y_mm = y;
    return true;
}
