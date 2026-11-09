/**
 * @file
 * @ingroup drv_lh2_geometry
 *
 * @brief  LH2 camera point and floor lines
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdint.h>

#include "lh2_geometry.h"

//=========================== variables ========================================

static const uint32_t _periods[DB_LH2_GEOMETRY_STATIONS] = {
    959000,
    957000,
    953000,
    949000,
    947000,
    943000,
    941000,
    939000,
    937000,
    929000,
    919000,
    911000,
    907000,
    901000,
    893000,
    887000,
};

//=========================== private ==========================================

static double _angle(uint32_t count, uint8_t station) {
    return ((double)count * (double)DB_LH2_PERIOD_TICKS_PER_COUNT / _periods[station]) * 2.0 * M_PI;
}

/// Image line, l . (x, y, 1) = 0, of the sweep at rotor angle alpha. The
/// earlier sweep of the pair is the plane tilted one way, the later one the
/// other; sign is that of cos(azimuth) at the pair, which fixes the side the
/// pinhole image is on.
static void _image_line(double alpha, int earlier, double sign, double l[3]) {
    double t = tan(M_PI / 6);
    if (earlier) {
        double beta = alpha + M_PI / 3;
        l[0]        = -sign * cos(beta);
        l[1]        = t;
        l[2]        = -sign * sin(beta);
    } else {
        double gamma = alpha - M_PI / 3;
        l[0]         = sign * cos(gamma);
        l[1]         = t;
        l[2]         = sign * sin(gamma);
    }
}

/// adj(H), so that adj(H) H = det(H) I
static double _adjugate(const float h[3][3], double adj[3][3]) {
    double a = h[0][0], b = h[0][1], c = h[0][2];
    double d = h[1][0], e = h[1][1], f = h[1][2];
    double g = h[2][0], k = h[2][1], m = h[2][2];
    adj[0][0] = e * m - f * k;
    adj[0][1] = c * k - b * m;
    adj[0][2] = b * f - c * e;
    adj[1][0] = f * g - d * m;
    adj[1][1] = a * m - c * g;
    adj[1][2] = c * d - a * f;
    adj[2][0] = d * k - e * g;
    adj[2][1] = b * g - a * k;
    adj[2][2] = a * e - b * d;
    return a * adj[0][0] + b * adj[1][0] + c * adj[2][0];
}

/// The floor line of image line l, through the floor point of image point x.
/// L = adj(H)^T l, so L . (H x) = det(H) l . x. The signed distance of that
/// point to the line at alpha + da moves by det(H) (dl . x) / (w r) da, with
/// dl = (-l2, 0, l0) the derivative of the image line, w the third coordinate
/// of H x and r the norm of L's first two; the line itself moves the other
/// way along L. The normal is turned to point the way the line moves as
/// alpha grows.
static int _floor_line(const double adj[3][3], double det, const float h[3][3], const double l[3], const double x[3], db_lh2_floor_line_t *out) {
    double L[3];
    for (int j = 0; j < 3; j++) {
        L[j] = adj[0][j] * l[0] + adj[1][j] * l[1] + adj[2][j] * l[2];
    }
    double r = sqrt(L[0] * L[0] + L[1] * L[1]);
    double w = h[2][0] * x[0] + h[2][1] * x[1] + h[2][2] * x[2];
    if (!(r > 0) || !(fabs(w) > 0)) {
        return 0;
    }
    double dl_x  = -l[2] * x[0] + l[0] * x[2];
    double rate  = det * dl_x / (w * r);
    double sigma = fabs(rate) * (double)DB_LH2_SWEEP_SIGMA_RAD;
    double sign  = (rate > 0) ? -1.0 : 1.0;
    double nx    = sign * L[0] / r;
    double ny    = sign * L[1] / r;
    double d     = -sign * L[2] / r;
    double var   = sigma * sigma;
    if (!isfinite(nx) || !isfinite(ny) || !isfinite(d) || !isfinite(var) || !(var > 0)) {
        return 0;
    }
    out->nx      = (float)nx;
    out->ny      = (float)ny;
    out->d_mm    = (float)d;
    out->var_mm2 = (float)var;
    return 1;
}

//=========================== public ===========================================

uint32_t db_lh2_period(uint8_t station) {
    return (station < DB_LH2_GEOMETRY_STATIONS) ? _periods[station] : 0;
}

void db_lh2_camera_point(uint32_t count1, uint32_t count2, uint8_t station, double cam[2]) {
    if (station >= DB_LH2_GEOMETRY_STATIONS) {
        cam[0] = NAN;
        cam[1] = NAN;
        return;
    }
    double alpha_1 = _angle(count1, station);
    double alpha_2 = _angle(count2, station);

    double cam_x = -tan(0.5 * (alpha_1 + alpha_2));
    double cam_y = 0;

    if (count1 < count2) {
        cam_y = -sin(alpha_2 / 2 - alpha_1 / 2 - 60 * M_PI / 180) / tan(M_PI / 6);
    } else {
        cam_y = -sin(alpha_1 / 2 - alpha_2 / 2 - 60 * M_PI / 180) / tan(M_PI / 6);
    };
    // cam_y so far is -tan(elevation); the pinhole image point divides it by cos(azimuth)
    cam_y *= sqrt(1 + cam_x * cam_x);

    cam[0] = cam_x;
    cam[1] = cam_y;
}

uint8_t db_lh2_sweep_lines(uint32_t count1, uint32_t count2, uint8_t station, const float homography[3][3], db_lh2_floor_line_t out[2]) {
    if (station >= DB_LH2_GEOMETRY_STATIONS) {
        return 0;
    }
    double alpha_1 = _angle(count1, station);
    double alpha_2 = _angle(count2, station);
    double c       = cos(0.5 * (alpha_1 + alpha_2));
    if (!(fabs(c) > 0)) {
        return 0;
    }
    double sign = (c > 0) ? 1.0 : -1.0;

    double adj[3][3];
    double det = _adjugate(homography, adj);
    if (!isfinite(det) || !(fabs(det) > 0)) {
        return 0;
    }

    double cam[2];
    db_lh2_camera_point(count1, count2, station, cam);
    double x[3] = { cam[0], cam[1], 1.0 };

    double l1[3], l2[3];
    _image_line(alpha_1, count1 < count2, sign, l1);
    _image_line(alpha_2, count1 >= count2, sign, l2);

    db_lh2_floor_line_t lines[2] = { 0 };
    if (!_floor_line(adj, det, homography, l1, x, &lines[0]) || !_floor_line(adj, det, homography, l2, x, &lines[1])) {
        return 0;
    }
    for (uint8_t i = 0; i < 2; i++) {
        lines[i].station = station;
        lines[i].sweep   = i;
        out[i]           = lines[i];
    }
    return 2;
}
