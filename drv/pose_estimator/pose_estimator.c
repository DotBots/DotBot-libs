/**
 * @file
 * @ingroup drv_pose_estimator
 *
 * @brief  Extended Kalman filter on [x, y, heading]
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "geometry.h"
#include "pose_estimator.h"

//=========================== defines ==========================================

#define DEG_TO_RAD ((float)M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / (float)M_PI)

//=========================== private ==========================================

static float _wrap(float a) {
    while (a >= (float)M_PI) {
        a -= 2.0f * (float)M_PI;
    }
    while (a < -(float)M_PI) {
        a += 2.0f * (float)M_PI;
    }
    return a;
}

/// Photodiode offset from the axle midpoint, in the world frame, at heading theta
static void _lever(const db_pose_estimator_conf_t *conf, float theta, float *lx, float *ly) {
    float a = theta + conf->lever_angle_deg * DEG_TO_RAD;
    *lx     = -conf->lever_mm * sinf(a);
    *ly     = conf->lever_mm * cosf(a);
}

static void _travel_clear(db_pose_estimator_t *est) {
    memset(&est->travel, 0, sizeof(est->travel));
}

static void _chain_start(db_pose_estimator_t *est, float x_mm, float y_mm) {
    est->chain_x         = x_mm;
    est->chain_y         = y_mm;
    est->chain_bx        = 0;
    est->chain_by        = 0;
    est->chain_dtheta    = 0;
    est->chain_var_theta = 0;
    est->chain_count     = 1;
}

/// Adds a fix to the seed chain, and sets the pose once the chain can solve for
/// heading. The fix and the chain's first fix are the photodiode at two times;
/// odometry gives the axle travel b and rotation dtheta between them in the
/// body frame of the first, so z - z0 = Rot(theta0) (b + Rot(dtheta) l - l)
/// with l the lever in the body frame. Lengths on both sides match whatever
/// theta0 is, which is the consistency test; their angles differ by theta0.
static db_pose_estimator_result_t _chain_add(db_pose_estimator_t *est, float x_mm, float y_mm) {
    const db_pose_estimator_conf_t *conf = est->conf;
    if (est->chain_count == 0) {
        _chain_start(est, x_mm, y_mm);
        return DB_POSE_ESTIMATOR_CHAINED;
    }

    float l0x, l0y, l1x, l1y;
    _lever(conf, 0, &l0x, &l0y);
    _lever(conf, est->chain_dtheta, &l1x, &l1y);
    float vx       = est->chain_bx + l1x - l0x;
    float vy       = est->chain_by + l1y - l0y;
    float zx       = x_mm - est->chain_x;
    float zy       = y_mm - est->chain_y;
    float body_mm  = sqrtf(vx * vx + vy * vy);
    float world_mm = sqrtf(zx * zx + zy * zy);

    if (fabsf(world_mm - body_mm) > conf->seed_tolerance_mm) {
        _chain_start(est, x_mm, y_mm);
        return DB_POSE_ESTIMATOR_REJECTED;
    }
    if (est->chain_count < UINT32_MAX) {
        est->chain_count++;
    }
    if (est->chain_count < conf->seed_fixes || body_mm < conf->acquire_mm) {
        return DB_POSE_ESTIMATOR_CHAINED;
    }

    // Body-frame vectors turn to the world frame by Rot(theta), with body-forward
    // (0, 1); atan2 differences are therefore heading differences.
    float theta0 = atan2f(zy, zx) - atan2f(vy, vx);
    float theta  = _wrap(theta0 + est->chain_dtheta);
    float lx, ly;
    _lever(conf, theta, &lx, &ly);

    float var_theta = 2.0f * conf->r_pos_mm2 / (body_mm * body_mm) + est->chain_var_theta;
    // d(axle)/d(theta) = -d(lever)/d(theta) = (ly, -lx)
    float jx = ly;
    float jy = -lx;

    est->x                  = x_mm - lx;
    est->y                  = y_mm - ly;
    est->theta              = theta;
    est->P[0][0]            = conf->r_pos_mm2 + jx * jx * var_theta;
    est->P[0][1]            = jx * jy * var_theta;
    est->P[1][0]            = est->P[0][1];
    est->P[1][1]            = conf->r_pos_mm2 + jy * jy * var_theta;
    est->P[0][2]            = jx * var_theta;
    est->P[2][0]            = est->P[0][2];
    est->P[1][2]            = jy * var_theta;
    est->P[2][1]            = est->P[1][2];
    est->P[2][2]            = var_theta;
    est->status             = DB_POSE_ESTIMATOR_TRACKING;
    est->chain_count        = 0;
    est->ticks_since_accept = 0;
    _travel_clear(est);
    est->seeds++;
    return DB_POSE_ESTIMATOR_SEEDED;
}

/// Gated EKF update with h(x) = axle + lever(theta), on the fix moved forward
/// by the photodiode travel since it was captured
static db_pose_estimator_result_t _gated_update(db_pose_estimator_t *est, float x_mm, float y_mm) {
    const db_pose_estimator_conf_t *conf = est->conf;
    float(*P)[3]                         = est->P;

    uint32_t age = (conf->fix_age_ticks < DB_POSE_ESTIMATOR_FIX_AGE_MAX) ? conf->fix_age_ticks : DB_POSE_ESTIMATOR_FIX_AGE_MAX;
    for (uint32_t i = 1; i <= age; i++) {
        uint32_t slot = (est->travel.head + DB_POSE_ESTIMATOR_FIX_AGE_MAX - i) % DB_POSE_ESTIMATOR_FIX_AGE_MAX;
        x_mm += est->travel.x[slot];
        y_mm += est->travel.y[slot];
    }

    float lx, ly;
    _lever(conf, est->theta, &lx, &ly);
    float y0 = x_mm - (est->x + lx);
    float y1 = y_mm - (est->y + ly);

    // H = [[1, 0, a], [0, 1, b]] with (a, b) = d(lever)/d(theta)
    float a = -ly;
    float b = lx;

    float pht[3][2];
    for (int i = 0; i < 3; i++) {
        pht[i][0] = P[i][0] + a * P[i][2];
        pht[i][1] = P[i][1] + b * P[i][2];
    }
    float s00 = pht[0][0] + a * pht[2][0] + conf->r_pos_mm2;
    float s01 = pht[0][1] + a * pht[2][1];
    float s10 = pht[1][0] + b * pht[2][0];
    float s11 = pht[1][1] + b * pht[2][1] + conf->r_pos_mm2;
    float det = s00 * s11 - s01 * s10;
    if (!(det > 0)) {
        return DB_POSE_ESTIMATOR_REJECTED;
    }
    float i00 = s11 / det;
    float i01 = -s01 / det;
    float i10 = -s10 / det;
    float i11 = s00 / det;

    float d2     = y0 * (i00 * y0 + i01 * y1) + y1 * (i10 * y0 + i11 * y1);
    est->last_d2 = d2;
    if (d2 > conf->gate) {
        return DB_POSE_ESTIMATOR_REJECTED;
    }

    float k[3][2];
    for (int i = 0; i < 3; i++) {
        k[i][0] = pht[i][0] * i00 + pht[i][1] * i10;
        k[i][1] = pht[i][0] * i01 + pht[i][1] * i11;
    }
    est->x += k[0][0] * y0 + k[0][1] * y1;
    est->y += k[1][0] * y0 + k[1][1] * y1;
    est->theta = _wrap(est->theta + k[2][0] * y0 + k[2][1] * y1);

    // P -= K (H P) with H P = pht transposed
    float np[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            np[i][j] = P[i][j] - (k[i][0] * pht[j][0] + k[i][1] * pht[j][1]);
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = 0.5f * (np[i][j] + np[j][i]);
        }
    }
    return DB_POSE_ESTIMATOR_ACCEPTED;
}

//=========================== public ===========================================

void db_pose_estimator_init(db_pose_estimator_t *est, const db_pose_estimator_conf_t *conf) {
    memset(est, 0, sizeof(*est));
    est->conf   = conf;
    est->status = DB_POSE_ESTIMATOR_SEEDING;
}

void db_pose_estimator_seed(db_pose_estimator_t *est, float x_mm, float y_mm, float heading_deg, float heading_sd_deg) {
    memset(est->P, 0, sizeof(est->P));
    est->x                  = x_mm;
    est->y                  = y_mm;
    est->theta              = _wrap(heading_deg * DEG_TO_RAD);
    est->P[0][0]            = est->conf->r_pos_mm2;
    est->P[1][1]            = est->conf->r_pos_mm2;
    est->P[2][2]            = (heading_sd_deg * DEG_TO_RAD) * (heading_sd_deg * DEG_TO_RAD);
    est->status             = DB_POSE_ESTIMATOR_TRACKING;
    est->chain_count        = 0;
    est->ticks_since_accept = 0;
    _travel_clear(est);
}

void db_pose_estimator_predict(db_pose_estimator_t *est, int32_t counts_left, int32_t counts_right, uint32_t elapsed_ticks) {
    const db_pose_estimator_conf_t *conf = est->conf;
    est->predicts++;

    if (UINT32_MAX - est->ticks_since_accept < elapsed_ticks) {
        est->ticks_since_accept = UINT32_MAX;
    } else {
        est->ticks_since_accept += elapsed_ticks;
    }
    if (est->status == DB_POSE_ESTIMATOR_TRACKING && est->ticks_since_accept > conf->timeout_ticks) {
        est->status      = DB_POSE_ESTIMATOR_LOST;
        est->chain_count = 0;
    }

    float d_left  = (float)counts_left * DB_MM_PER_COUNT;
    float d_right = (float)counts_right * DB_MM_PER_COUNT;
    float d       = 0.5f * (d_left + d_right);
    float dd      = fabsf(d_right - d_left);
    float dtheta  = -(d_right - d_left) / db_track_effective_mm(d_left, d_right);

    float turn_scale = 1.0f;
    if (dd > 0 && conf->turn_speed_ref_mm_s > 0) {
        float dt_s  = (float)(elapsed_ticks ? elapsed_ticks : 1U) * (DB_POSE_ESTIMATOR_TICK_MS / 1000.0f);
        float ratio = (dd / dt_s) / conf->turn_speed_ref_mm_s;
        if (ratio > 1.0f) {
            turn_scale = ratio;
        }
    }
    float q_theta = (conf->q_heading_roll_deg2_per_mm * fabsf(d) + conf->q_heading_turn_deg2_per_mm * dd * turn_scale) * DEG_TO_RAD * DEG_TO_RAD;
    float q_pos   = conf->q_pos_mm2_per_mm * fabsf(d);

    if (est->status != DB_POSE_ESTIMATOR_TRACKING && est->chain_count > 0) {
        float mid = est->chain_dtheta + 0.5f * dtheta;
        est->chain_bx += -d * sinf(mid);
        est->chain_by += d * cosf(mid);
        est->chain_dtheta += dtheta;
        est->chain_var_theta += q_theta;
    }
    if (est->status == DB_POSE_ESTIMATOR_SEEDING) {
        return;
    }

    float lx0, ly0, lx1, ly1;
    _lever(conf, est->theta, &lx0, &ly0);
    float mid = est->theta + 0.5f * dtheta;
    float c   = cosf(mid);
    float s   = sinf(mid);
    est->x += -d * s;
    est->y += d * c;
    est->theta = _wrap(est->theta + dtheta);
    _lever(conf, est->theta, &lx1, &ly1);
    est->travel.x[est->travel.head] = -d * s + lx1 - lx0;
    est->travel.y[est->travel.head] = d * c + ly1 - ly0;
    est->travel.head                = (est->travel.head + 1) % DB_POSE_ESTIMATOR_FIX_AGE_MAX;

    // F = [[1, 0, f0], [0, 1, f1], [0, 0, 1]]
    float(*P)[3] = est->P;
    float f0     = -d * c;
    float f1     = -d * s;
    float fp[3][3];
    for (int j = 0; j < 3; j++) {
        fp[0][j] = P[0][j] + f0 * P[2][j];
        fp[1][j] = P[1][j] + f1 * P[2][j];
        fp[2][j] = P[2][j];
    }
    for (int i = 0; i < 3; i++) {
        P[i][0] = fp[i][0] + fp[i][2] * f0;
        P[i][1] = fp[i][1] + fp[i][2] * f1;
        P[i][2] = fp[i][2];
    }
    P[0][0] += q_pos;
    P[1][1] += q_pos;
    P[2][2] += q_theta;
}

db_pose_estimator_result_t db_pose_estimator_update(db_pose_estimator_t *est, float x_mm, float y_mm) {
    db_pose_estimator_result_t result;
    if (est->status == DB_POSE_ESTIMATOR_SEEDING) {
        result = _chain_add(est, x_mm, y_mm);
    } else {
        result = _gated_update(est, x_mm, y_mm);
        if (result == DB_POSE_ESTIMATOR_ACCEPTED) {
            est->status             = DB_POSE_ESTIMATOR_TRACKING;
            est->ticks_since_accept = 0;
            est->chain_count        = 0;
        } else if (est->status == DB_POSE_ESTIMATOR_LOST) {
            result = _chain_add(est, x_mm, y_mm);
            if (result == DB_POSE_ESTIMATOR_CHAINED) {
                result = DB_POSE_ESTIMATOR_REJECTED;
            }
        }
    }
    if (result == DB_POSE_ESTIMATOR_ACCEPTED) {
        est->accepted++;
    } else if (result == DB_POSE_ESTIMATOR_REJECTED) {
        est->rejected++;
    }
    return result;
}

bool db_pose_estimator_heading_deg(const db_pose_estimator_t *est, float *deg) {
    if (est->status != DB_POSE_ESTIMATOR_TRACKING) {
        return false;
    }
    float d = est->theta * RAD_TO_DEG;
    if (d >= 180.0f) {
        d -= 360.0f;
    }
    *deg = d;
    return true;
}

bool db_pose_estimator_sensor(const db_pose_estimator_t *est, float *x_mm, float *y_mm) {
    if (est->status != DB_POSE_ESTIMATOR_TRACKING) {
        return false;
    }
    float lx, ly;
    _lever(est->conf, est->theta, &lx, &ly);
    *x_mm = est->x + lx;
    *y_mm = est->y + ly;
    return true;
}
