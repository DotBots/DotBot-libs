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

static uint32_t _fix_age(const db_pose_estimator_conf_t *conf) {
    return (conf->fix_age_ticks < DB_POSE_ESTIMATOR_FIX_AGE_MAX) ? conf->fix_age_ticks : DB_POSE_ESTIMATOR_FIX_AGE_MAX;
}

/// Ring slot of the i-th most recent predict, i from 1
static uint32_t _travel_slot(const db_pose_estimator_t *est, uint32_t i) {
    return (est->travel.head + DB_POSE_ESTIMATOR_FIX_AGE_MAX - i) % DB_POSE_ESTIMATOR_FIX_AGE_MAX;
}

/// Axle travel and rotation over the last age predicts, integrated back from
/// the current heading
static void _travel_recent(const db_pose_estimator_t *est, uint32_t age, float *ax, float *ay, float *dtheta) {
    float theta = est->theta;
    *ax         = 0;
    *ay         = 0;
    for (uint32_t i = 1; i <= age; i++) {
        uint32_t slot = _travel_slot(est, i);
        theta -= est->travel.dtheta[slot];
        float mid = theta + 0.5f * est->travel.dtheta[slot];
        *ax += -est->travel.d[slot] * sinf(mid);
        *ay += est->travel.d[slot] * cosf(mid);
    }
    *dtheta = est->theta - theta;
}

/// Moves the pose and its covariance over one odometry step
static void _propagate(db_pose_estimator_t *est, float d, float dtheta, float q_theta) {
    float mid = est->theta + 0.5f * dtheta;
    float c   = cosf(mid);
    float s   = sinf(mid);
    est->x += -d * s;
    est->y += d * c;
    est->theta = _wrap(est->theta + dtheta);

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
    float q_pos = est->conf->q_pos_mm2_per_mm * fabsf(d);
    P[0][0] += q_pos;
    P[1][1] += q_pos;
    P[2][2] += q_theta;
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
/// heading. The fix and the chain's first fix are the photodiode at two capture
/// times; odometry gives the axle travel b and rotation dtheta between them in
/// the body frame of the first, so z - z0 = Rot(theta0) (b + Rot(dtheta) l - l)
/// with l the lever in the body frame. Lengths on both sides match whatever
/// theta0 is, which is the consistency test; their angles differ by theta0.
/// The pose solved is the one at capture, carried forward over the fix age.
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
    est->kidnap_count       = 0;
    est->ticks_since_accept = 0;
    for (uint32_t i = _fix_age(conf); i >= 1; i--) {
        uint32_t slot = _travel_slot(est, i);
        _propagate(est, est->travel.d[slot], est->travel.dtheta[slot], est->travel.q_theta[slot]);
    }
    est->seeds++;
    return DB_POSE_ESTIMATOR_SEEDED;
}

/// Counts a fix the gate rejected while TRACKING toward a kidnap, and on the
/// last one returns to SEEDING with the chain started on these fixes
static void _kidnap_check(db_pose_estimator_t *est, float x_mm, float y_mm) {
    const db_pose_estimator_conf_t *conf = est->conf;
    if (conf->kidnap_fixes == 0) {
        return;
    }
    float dx = x_mm - est->kidnap_x;
    float dy = y_mm - est->kidnap_y;
    if (est->kidnap_count == 0 || est->kidnap_travel_mm > conf->kidnap_still_mm || sqrtf(dx * dx + dy * dy) > conf->seed_tolerance_mm) {
        est->kidnap_x         = x_mm;
        est->kidnap_y         = y_mm;
        est->kidnap_travel_mm = 0;
        est->kidnap_count     = 1;
    } else {
        est->kidnap_count++;
    }
    if (est->kidnap_count < conf->kidnap_fixes) {
        return;
    }
    est->status = DB_POSE_ESTIMATOR_SEEDING;
    _chain_start(est, est->kidnap_x, est->kidnap_y);
    est->chain_count  = est->kidnap_count;
    est->kidnap_count = 0;
    est->kidnaps++;
}

/// Gated EKF update with h(x) = axle + lever(theta), on the fix moved forward
/// by the photodiode travel since it was captured
static db_pose_estimator_result_t _gated_update(db_pose_estimator_t *est, float x_mm, float y_mm) {
    const db_pose_estimator_conf_t *conf = est->conf;
    float(*P)[3]                         = est->P;

    float ax, ay, dtheta, lx0, ly0, lx, ly;
    _travel_recent(est, _fix_age(conf), &ax, &ay, &dtheta);
    _lever(conf, est->theta - dtheta, &lx0, &ly0);
    _lever(conf, est->theta, &lx, &ly);
    x_mm += ax + lx - lx0;
    y_mm += ay + ly - ly0;

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

    // Joseph form, P = (I - K H) P (I - K H)^T + K R K^T, which stays positive
    // definite in float32 as fixes on a still robot shrink P toward rank 1
    float ikh[3][3];
    for (int i = 0; i < 3; i++) {
        ikh[i][0] = (i == 0) - k[i][0];
        ikh[i][1] = (i == 1) - k[i][1];
        ikh[i][2] = (i == 2) - (k[i][0] * a + k[i][1] * b);
    }
    float ap[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ap[i][j] = ikh[i][0] * P[0][j] + ikh[i][1] * P[1][j] + ikh[i][2] * P[2][j];
        }
    }
    float np[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j <= i; j++) {
            np[i][j] = ap[i][0] * ikh[j][0] + ap[i][1] * ikh[j][1] + ap[i][2] * ikh[j][2] + conf->r_pos_mm2 * (k[i][0] * k[j][0] + k[i][1] * k[j][1]);
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j <= i; j++) {
            P[i][j] = np[i][j];
            P[j][i] = np[i][j];
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
    est->kidnap_count       = 0;
    est->ticks_since_accept = 0;
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
        est->status       = DB_POSE_ESTIMATOR_LOST;
        est->chain_count  = 0;
        est->kidnap_count = 0;
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

    if (est->kidnap_count > 0) {
        est->kidnap_travel_mm += fabsf(d_left) + fabsf(d_right);
    }

    // The chain runs fix_age behind, on the step leaving the age window, so its
    // odometry spans the capture times of its fixes
    uint32_t age        = _fix_age(conf);
    float    old_d      = d;
    float    old_dtheta = dtheta;
    float    old_q      = q_theta;
    if (age > 0) {
        uint32_t slot = _travel_slot(est, age);
        old_d         = est->travel.d[slot];
        old_dtheta    = est->travel.dtheta[slot];
        old_q         = est->travel.q_theta[slot];
    }
    if (est->status != DB_POSE_ESTIMATOR_TRACKING && est->chain_count > 0) {
        float mid = est->chain_dtheta + 0.5f * old_dtheta;
        est->chain_bx += -old_d * sinf(mid);
        est->chain_by += old_d * cosf(mid);
        est->chain_dtheta += old_dtheta;
        est->chain_var_theta += old_q;
    }
    est->travel.d[est->travel.head]       = d;
    est->travel.dtheta[est->travel.head]  = dtheta;
    est->travel.q_theta[est->travel.head] = q_theta;
    est->travel.head                      = (est->travel.head + 1) % DB_POSE_ESTIMATOR_FIX_AGE_MAX;

    if (est->status != DB_POSE_ESTIMATOR_SEEDING) {
        _propagate(est, d, dtheta, q_theta);
    }
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
            est->kidnap_count       = 0;
        } else if (est->status == DB_POSE_ESTIMATOR_TRACKING) {
            _kidnap_check(est, x_mm, y_mm);
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
