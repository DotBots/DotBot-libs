#ifndef __LH2_SELECT_H
#define __LH2_SELECT_H

/**
 * @defgroup    drv_lh2_select      LH2 station choice
 * @ingroup     drv
 * @brief       From the pairs of one fix to one station's position and every station's floor lines
 *
 * Each calibrated station with a fresh pair is solved through its homography.
 * A non-finite solve, or one outside its own station's rectangle, is dropped;
 * of the rest, the station the previous fix kept stays while its solve is
 * inside its rectangle, and otherwise the one nearest its rectangle's centre
 * is kept, a tie keeping the one offered first, so offering in index order
 * keeps the lower index. Every solve inside its rectangle adds its pair's two
 * floor lines to a ring the caller drains; a station whose lines went in last
 * time keeps them in until its solve leaves the rectangle by margin_mm.
 *
 * No hardware calls and no module state: the caller owns the configuration,
 * the state carried from one fix to the next, and the ring.
 *
 * @{
 * @file
 * @copyright Inria, 2026
 * @}
 */

#include <stdbool.h>
#include <stdint.h>

#include "lh2_decoder.h"
#include "lh2_geometry.h"

//=========================== defines ==========================================

#define DB_LH2_SELECT_STATIONS         (16U)      ///< station indices 0 to 15
#define DB_LH2_SELECT_RECT_LEN         (4U)       ///< x_min, y_min, x_max, y_max, mm
#define DB_LH2_SELECT_RECT_MAX_DEFAULT (100000U)  ///< mm; x_max and y_max of the rectangle an all-0xFF one reads as
#define DB_LH2_SELECT_LINES_MAX        (8U)       ///< lines the ring holds: four stations, two sweeps each
#define DB_LH2_SELECT_MARGIN_MM        (20U)      ///< db_lh2_select_conf_load()'s margin_mm

/// The calibration as the choice sees it
typedef struct {
    uint16_t station_mask;                                             ///< bit i: station i has a homography and a rectangle
    uint32_t rect_mm[DB_LH2_SELECT_STATIONS][DB_LH2_SELECT_RECT_LEN];  ///< resolved, by station
    float    homography[DB_LH2_SELECT_STATIONS][3][3];                 ///< camera point to floor mm, by station
    uint16_t margin_mm;                                                ///< a station whose lines are in keeps them until its solve is this far outside; 0 for none
    bool     hold_station;                                             ///< the previous fix's station stays while its solve is inside its rectangle
} db_lh2_select_conf_t;

/// What the choice remembers from one fix to the next, owned by the caller
typedef struct {
    int8_t   kept;    ///< the station the last fix kept, -1 for none
    uint16_t inside;  ///< bit i: station i's lines went in on its last solve
} db_lh2_select_state_t;

/// The solve kept so far for one fix
typedef struct {
    bool    found;    ///< a solve has been kept
    uint8_t station;  ///< its station
    double  x_mm;     ///< mm
    double  y_mm;     ///< mm
    double  d2;       ///< squared distance to its station's rectangle centre, mm^2
    bool    held;     ///< the solve is the previous fix's station's, held over a nearer one
} db_lh2_fix_t;

/// Floor lines not yet drained, oldest first; a pair's two lines are never split
typedef struct {
    db_lh2_floor_line_t line[DB_LH2_SELECT_LINES_MAX];
    uint8_t             head;   ///< next line written, always even
    uint8_t             count;  ///< lines held
} db_lh2_lines_t;

/// What one offered pair did
typedef enum {
    DB_LH2_SELECT_KEPT,          ///< inside its rectangle and now the fix; its lines pushed
    DB_LH2_SELECT_INSIDE,        ///< inside its rectangle, but an earlier solve is nearer its own centre; its lines pushed
    DB_LH2_SELECT_OUTSIDE,       ///< finite but outside its rectangle
    DB_LH2_SELECT_NOT_FINITE,    ///< the solve is NaN or infinite
    DB_LH2_SELECT_UNCALIBRATED,  ///< the station is not in the mask; not solved
} db_lh2_select_result_t;

//=========================== prototypes =======================================

/// The rectangle a stored one stands for: all 0xFF is (0, 0, default, default)
void db_lh2_select_rect_resolve(const uint32_t stored[DB_LH2_SELECT_RECT_LEN], uint32_t out[DB_LH2_SELECT_RECT_LEN]);

/// Whether (x, y) is inside rect, edges included; NaN is outside
bool db_lh2_select_rect_contains(const uint32_t rect[DB_LH2_SELECT_RECT_LEN], double x, double y);

/**
 * @brief   Load the stations of a calibration
 *
 * Station i's homography and resolved rectangle are copied for every bit i of
 * station_mask; the other stations keep what they held. The mask replaces the
 * previous one; margin_mm and hold_station are set to their defaults,
 * DB_LH2_SELECT_MARGIN_MM and true.
 *
 * @param[in,out]  conf          the configuration
 * @param[in]      station_mask  bit i: station i is calibrated; bits past 15 are ignored
 * @param[in]      homographies  by station
 * @param[in]      stored_rects  by station, as stored; see db_lh2_select_rect_resolve()
 */
void db_lh2_select_conf_load(db_lh2_select_conf_t *conf, uint32_t station_mask, const float homographies[][3][3], const uint32_t stored_rects[][DB_LH2_SELECT_RECT_LEN]);

/// No station kept, no lines in
void db_lh2_select_state_init(db_lh2_select_state_t *state);

/// Start a fix with nothing kept
void db_lh2_select_begin(db_lh2_fix_t *fix);

/// End a fix: remember its station for the next one
void db_lh2_select_end(db_lh2_select_state_t *state, const db_lh2_fix_t *fix);

/**
 * @brief   Offer one solve, already gated, to a fix
 *
 * @param[in,out]  fix      the fix being built
 * @param[in]      station  its station
 * @param[in]      x        mm
 * @param[in]      y        mm
 * @param[in]      rect     its station's resolved rectangle
 *
 * @return  whether it is now the one kept; never when (x, y) is outside rect
 */
bool db_lh2_select_offer_solve(db_lh2_fix_t *fix, uint8_t station, double x, double y, const uint32_t rect[DB_LH2_SELECT_RECT_LEN]);

/**
 * @brief   Offer one station's pair to a fix: solve, gate, choose, and push its lines
 *
 * @param[in]      conf   the calibration
 * @param[in,out]  state  what the previous fixes left
 * @param[in,out]  fix    the fix being built
 * @param[in,out]  lines  the ring a solve inside its rectangle pushes its two lines into
 * @param[in]      pair   the station's two counts
 * @param[out]     xy_mm  the solve, NaN when the station is uncalibrated
 *
 * @return  what the pair did; INSIDE also for a solve outside its rectangle but within the margin
 */
db_lh2_select_result_t db_lh2_select_offer_pair(const db_lh2_select_conf_t *conf, db_lh2_select_state_t *state, db_lh2_fix_t *fix, db_lh2_lines_t *lines, const db_lh2_pair_t *pair, double xy_mm[2]);

/**
 * @brief   One whole fix: offer every pair in order
 *
 * @param[in]      conf   the calibration
 * @param[in,out]  state  what the previous fixes left, updated
 * @param[in]      pairs  the fresh pairs, in station order
 * @param[in]      n      how many
 * @param[out]     fix    the fix
 * @param[in,out]  lines  the ring
 *
 * @return  whether a solve was kept
 */
bool db_lh2_select_fix(const db_lh2_select_conf_t *conf, db_lh2_select_state_t *state, const db_lh2_pair_t *pairs, uint8_t n, db_lh2_fix_t *fix, db_lh2_lines_t *lines);

/// Empty the ring
void db_lh2_lines_init(db_lh2_lines_t *lines);

/// Push one pair's two lines, overwriting the oldest pair when full
void db_lh2_lines_push(db_lh2_lines_t *lines, const db_lh2_floor_line_t pair[2]);

/// Move up to max lines out, oldest first; returns how many
uint8_t db_lh2_lines_drain(db_lh2_lines_t *lines, db_lh2_floor_line_t *out, uint8_t max);

/**
 * @brief   Take every station's pair that is ready, calibrated or not, in station order
 *
 * @param[in,out]  dec  the decoder
 * @param[out]     out  the pairs taken
 * @param[in]      max  at most this many; the stations past it stay ready
 *
 * @return  how many were taken
 */
uint8_t db_lh2_select_take_pairs(db_lh2_decoder_t *dec, db_lh2_pair_t *out, uint8_t max);

#endif
