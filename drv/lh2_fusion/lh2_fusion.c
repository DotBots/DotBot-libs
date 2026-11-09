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

//=========================== private ==========================================

static bool _well_formed(const db_lh2_floor_line_t *l) {
    float n2 = l->nx * l->nx + l->ny * l->ny;
    return isfinite(n2) && isfinite(l->d_mm) && isfinite(l->var_mm2) && l->var_mm2 > 0 && n2 > 1e-12f;
}

/// Weighted intersection of the well-formed lines other than lines[skip]
static bool _intersect(const db_lh2_floor_line_t *lines, uint8_t count, uint8_t skip, float *x_mm, float *y_mm) {
    // Normal equations A p = b of sum w (n . p - d)^2, with unit normals
    float a00 = 0, a01 = 0, a11 = 0, b0 = 0, b1 = 0;
    for (uint8_t i = 0; i < count; i++) {
        const db_lh2_floor_line_t *l = &lines[i];
        if (i == skip || !_well_formed(l)) {
            continue;
        }
        float n2 = l->nx * l->nx + l->ny * l->ny;
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

/// Largest squared distance, in variances, from (x, y) to a well-formed line other than lines[skip]
static float _worst_d2(const db_lh2_floor_line_t *lines, uint8_t count, uint8_t skip, float x_mm, float y_mm) {
    float worst = 0;
    for (uint8_t i = 0; i < count; i++) {
        const db_lh2_floor_line_t *l = &lines[i];
        if (i == skip || !_well_formed(l)) {
            continue;
        }
        float r = l->nx * x_mm + l->ny * y_mm - l->d_mm;
        worst   = fmaxf(worst, r * r / l->var_mm2);
    }
    return worst;
}

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
    return _intersect(lines, count, count, x_mm, y_mm);
}

uint8_t db_lh2_fusion_consensus(const db_lh2_floor_line_t *lines, uint8_t count, float *x_mm, float *y_mm) {
    uint8_t valid = 0;
    for (uint8_t i = 0; i < count; i++) {
        valid += _well_formed(&lines[i]);
    }
    float x, y;
    if (!_intersect(lines, count, count, &x, &y)) {
        return 0;
    }
    if (valid < 3 || _worst_d2(lines, count, count, x, y) <= DB_LH2_FUSION_GATE) {
        *x_mm = x;
        *y_mm = y;
        return valid;
    }
    // Three lines left out of four or more still check each other
    if (valid < 4) {
        return 0;
    }
    bool  found = false;
    float best  = DB_LH2_FUSION_GATE;
    for (uint8_t skip = 0; skip < count; skip++) {
        float sx, sy;
        if (!_well_formed(&lines[skip]) || !_intersect(lines, count, skip, &sx, &sy)) {
            continue;
        }
        float worst = _worst_d2(lines, count, skip, sx, sy);
        if (worst <= best) {
            best  = worst;
            x     = sx;
            y     = sy;
            found = true;
        }
    }
    if (!found) {
        return 0;
    }
    *x_mm = x;
    *y_mm = y;
    return valid - 1;
}
