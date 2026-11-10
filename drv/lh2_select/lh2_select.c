/**
 * @file
 * @ingroup drv_lh2_select
 *
 * @brief  LH2 station choice and floor lines
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <string.h>

#include "lh2_select.h"

_Static_assert(DB_LH2_SELECT_STATIONS == DB_LH2_STATIONS, "one station count");
_Static_assert(DB_LH2_SELECT_LINES_MAX % 2 == 0, "pairs never straddle the end of the ring");

//=========================== public ===========================================

void db_lh2_select_rect_resolve(const uint32_t stored[DB_LH2_SELECT_RECT_LEN], uint32_t out[DB_LH2_SELECT_RECT_LEN]) {
    bool absent = true;
    for (uint8_t i = 0; i < DB_LH2_SELECT_RECT_LEN; i++) {
        absent = absent && (stored[i] == UINT32_MAX);
    }
    if (absent) {
        out[0] = 0;
        out[1] = 0;
        out[2] = DB_LH2_SELECT_RECT_MAX_DEFAULT;
        out[3] = DB_LH2_SELECT_RECT_MAX_DEFAULT;
        return;
    }
    memcpy(out, stored, DB_LH2_SELECT_RECT_LEN * sizeof(out[0]));
}

bool db_lh2_select_rect_contains(const uint32_t rect[DB_LH2_SELECT_RECT_LEN], double x, double y) {
    // Every comparison with NaN is false, so a NaN coordinate is outside
    return x >= rect[0] && x <= rect[2] && y >= rect[1] && y <= rect[3];
}

void db_lh2_select_conf_load(db_lh2_select_conf_t *conf, uint32_t station_mask, const float homographies[][3][3], const uint32_t stored_rects[][DB_LH2_SELECT_RECT_LEN]) {
    for (uint8_t station = 0; station < DB_LH2_SELECT_STATIONS; station++) {
        if (((station_mask >> station) & 1U) == 0) {
            continue;
        }
        db_lh2_select_rect_resolve(stored_rects[station], conf->rect_mm[station]);
        memcpy(conf->homography[station], homographies[station], sizeof(conf->homography[0]));
    }
    conf->station_mask = (uint16_t)station_mask;
}

void db_lh2_select_begin(db_lh2_fix_t *fix) {
    memset(fix, 0, sizeof(*fix));
}

bool db_lh2_select_offer_solve(db_lh2_fix_t *fix, uint8_t station, double x, double y, const uint32_t rect[DB_LH2_SELECT_RECT_LEN]) {
    if (!db_lh2_select_rect_contains(rect, x, y)) {
        return false;
    }
    double dx = x - 0.5 * ((double)rect[0] + (double)rect[2]);
    double dy = y - 0.5 * ((double)rect[1] + (double)rect[3]);
    double d2 = dx * dx + dy * dy;
    if (fix->found && !(d2 < fix->d2)) {
        return false;
    }
    fix->found   = true;
    fix->station = station;
    fix->x_mm    = x;
    fix->y_mm    = y;
    fix->d2      = d2;
    return true;
}

db_lh2_select_result_t db_lh2_select_offer_pair(const db_lh2_select_conf_t *conf, db_lh2_fix_t *fix, db_lh2_lines_t *lines, const db_lh2_pair_t *pair, double xy_mm[2]) {
    uint8_t station = pair->station;
    if (station >= DB_LH2_SELECT_STATIONS || ((conf->station_mask >> station) & 1U) == 0) {
        xy_mm[0] = NAN;
        xy_mm[1] = NAN;
        return DB_LH2_SELECT_UNCALIBRATED;
    }
    db_lh2_solve(pair->count[0], pair->count[1], station, conf->homography[station], xy_mm);
    double x = xy_mm[0];
    double y = xy_mm[1];
    if (!isfinite(x) || !isfinite(y)) {
        return DB_LH2_SELECT_NOT_FINITE;
    }
    if (!db_lh2_select_rect_contains(conf->rect_mm[station], x, y)) {
        return DB_LH2_SELECT_OUTSIDE;
    }
    bool                kept = db_lh2_select_offer_solve(fix, station, x, y, conf->rect_mm[station]);
    db_lh2_floor_line_t pair_lines[2];
    if (db_lh2_sweep_lines(pair->count[0], pair->count[1], station, conf->homography[station], pair_lines) == 2) {
        db_lh2_lines_push(lines, pair_lines);
    }
    return kept ? DB_LH2_SELECT_KEPT : DB_LH2_SELECT_INSIDE;
}

bool db_lh2_select_fix(const db_lh2_select_conf_t *conf, const db_lh2_pair_t *pairs, uint8_t n, db_lh2_fix_t *fix, db_lh2_lines_t *lines) {
    db_lh2_select_begin(fix);
    for (uint8_t i = 0; i < n; i++) {
        double xy[2];
        db_lh2_select_offer_pair(conf, fix, lines, &pairs[i], xy);
    }
    return fix->found;
}

void db_lh2_lines_init(db_lh2_lines_t *lines) {
    memset(lines, 0, sizeof(*lines));
}

void db_lh2_lines_push(db_lh2_lines_t *lines, const db_lh2_floor_line_t pair[2]) {
    lines->line[lines->head]     = pair[0];
    lines->line[lines->head + 1] = pair[1];
    lines->head                  = (uint8_t)((lines->head + 2) % DB_LH2_SELECT_LINES_MAX);
    lines->count                 = (lines->count + 2U > DB_LH2_SELECT_LINES_MAX) ? DB_LH2_SELECT_LINES_MAX : (uint8_t)(lines->count + 2);
}

uint8_t db_lh2_lines_drain(db_lh2_lines_t *lines, db_lh2_floor_line_t *out, uint8_t max) {
    uint8_t n     = (max < lines->count) ? max : lines->count;
    uint8_t first = (uint8_t)((lines->head + DB_LH2_SELECT_LINES_MAX - lines->count) % DB_LH2_SELECT_LINES_MAX);
    for (uint8_t i = 0; i < n; i++) {
        out[i] = lines->line[(first + i) % DB_LH2_SELECT_LINES_MAX];
    }
    lines->count -= n;
    return n;
}

uint8_t db_lh2_select_take_pairs(db_lh2_decoder_t *dec, db_lh2_pair_t *out, uint8_t max) {
    uint8_t n = 0;
    for (uint8_t station = 0; station < DB_LH2_STATIONS && n < max; station++) {
        if (db_lh2_decoder_take_pair(dec, station, &out[n])) {
            n++;
        }
    }
    return n;
}
