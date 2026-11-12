/**
 * @file
 * @brief  Frozen copy of the LH2 station choice at swarmit a702520
 *
 * device/bootloader/Source/lh2_select.c and h, and from localization.c the
 * calibration load, the pair take, the floor-line ring, the fix and the
 * raw-count drain, as they stand at swarmit a702520 (0.12.0), with the solve
 * they called, db_lh2_calculate_position() and db_lh2_store_homography() of
 * bsp/nrf/lh2_default.c at DotBot-libs 228ce9a. Every copied body is
 * unchanged; the macros below are the hooks: the BSP's slots are an
 * lh2_ref_t the test fills, stopping and starting the capture does nothing,
 * printing evaluates its arguments and prints nothing, and the kept solve is
 * recorded as lh2_select_offer() changes it. The copy is the definition of
 * today's behaviour: it is never edited to follow drv/lh2_select.
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "lh2_geometry.h"
#include "lh2_reference.h"
#include "lh2_reference_select.h"

//=========================== hooks ============================================

#define LH2_BASESTATION_COUNT           LH2_REF_STATIONS
#define DB_LH2_PROCESSED_DATA_AVAILABLE LH2_REF_PROCESSED_DATA_AVAILABLE
#define DB_LH2_NO_NEW_DATA              LH2_REF_NO_NEW_DATA
#define db_lh2_t                        lh2_ref_t
#define db_lh2_stop()                   ((void)0)
#define db_lh2_start()                  ((void)0)
#define localization_start()            (true)
#define printf(...)                     _print_nothing(0, __VA_ARGS__)
#define position_2d_t                   lh2_ref_position_t
#define lh2_raw_sample_t                lh2_ref_raw_sample_t
#define localization_init               lh2_ref_select_init
#define localization_get_lines          lh2_ref_select_get_lines
#define localization_get_position       lh2_ref_select_get_position
#define localization_get_raw_counts     lh2_ref_select_get_raw_counts

static void _print_nothing(int unused, ...) {
    (void)unused;
}

//=========================== lh2_select.h at a702520 ==========================

// clang-format off
#define LH2_SELECT_STATIONS         (16U)       ///< station indices 0 to 15
#define LH2_SELECT_RECT_LEN         (4U)        ///< x_min, y_min, x_max, y_max, mm
#define LH2_SELECT_RECT_MAX_DEFAULT (100000U)   ///< mm; x_max and y_max of the rectangle an all-0xFF one reads as

/// The rectangle a stored one stands for: all 0xFF is (0, 0, default, default)
void lh2_select_rect_resolve(const uint32_t stored[LH2_SELECT_RECT_LEN], uint32_t out[LH2_SELECT_RECT_LEN]);

/// Whether (x, y) is finite and inside rect, edges included
bool lh2_select_rect_contains(const uint32_t rect[LH2_SELECT_RECT_LEN], double x, double y);

/// The solve kept so far for one fix
typedef struct {
    bool    found;      ///< a solve has been kept
    uint8_t station;    ///< its station
    double  x;          ///< mm
    double  y;          ///< mm
    double  d2;         ///< squared distance to its station's rectangle centre, mm^2
} lh2_select_best_t;

/// Start a fix with nothing kept
void lh2_select_reset(lh2_select_best_t *best);

/// Offer one station's solve; returns whether it is now the one kept
bool lh2_select_offer(lh2_select_best_t *best, uint8_t station, double x, double y, const uint32_t rect[LH2_SELECT_RECT_LEN]);
// clang-format on

static lh2_select_best_t _best_seen;  ///< the hook's record of the kept solve

static void _hook_reset(lh2_select_best_t *best);
static bool _hook_offer(lh2_select_best_t *best, uint8_t station, double x, double y, const uint32_t rect[LH2_SELECT_RECT_LEN]);

//=========================== lh2_select.c at a702520 ==========================

// clang-format off
void lh2_select_rect_resolve(const uint32_t stored[LH2_SELECT_RECT_LEN], uint32_t out[LH2_SELECT_RECT_LEN]) {
    bool absent = true;
    for (uint8_t i = 0; i < LH2_SELECT_RECT_LEN; i++) {
        absent = absent && (stored[i] == UINT32_MAX);
    }
    if (absent) {
        out[0] = 0;
        out[1] = 0;
        out[2] = LH2_SELECT_RECT_MAX_DEFAULT;
        out[3] = LH2_SELECT_RECT_MAX_DEFAULT;
        return;
    }
    memcpy(out, stored, LH2_SELECT_RECT_LEN * sizeof(out[0]));
}

bool lh2_select_rect_contains(const uint32_t rect[LH2_SELECT_RECT_LEN], double x, double y) {
    // Every comparison with NaN is false, so a NaN coordinate is outside
    return x >= rect[0] && x <= rect[2] && y >= rect[1] && y <= rect[3];
}

void lh2_select_reset(lh2_select_best_t *best) {
    memset(best, 0, sizeof(*best));
}

bool lh2_select_offer(lh2_select_best_t *best, uint8_t station, double x, double y, const uint32_t rect[LH2_SELECT_RECT_LEN]) {
    if (!lh2_select_rect_contains(rect, x, y)) {
        return false;
    }
    double dx = x - 0.5 * ((double)rect[0] + (double)rect[2]);
    double dy = y - 0.5 * ((double)rect[1] + (double)rect[3]);
    double d2 = dx * dx + dy * dy;
    if (best->found && !(d2 < best->d2)) {
        return false;
    }
    best->found   = true;
    best->station = station;
    best->x       = x;
    best->y       = y;
    best->d2      = d2;
    return true;
}
// clang-format on

#define lh2_select_reset _hook_reset
#define lh2_select_offer _hook_offer

//=========================== lh2_default.c at DotBot-libs 228ce9a =============

// clang-format off
static double homography_matrix[LH2_BASESTATION_COUNT][3][3] = { 0 };

void db_lh2_calculate_position(uint32_t count1, uint32_t count2, uint32_t basestation_index, double *coordinates) {
    if (basestation_index >= LH2_BASESTATION_COUNT) {
        coordinates[0] = NAN;
        coordinates[1] = NAN;
        return;
    }
    double cam[2];
    db_lh2_camera_point(count1, count2, (uint8_t)basestation_index, cam);
    double cam_x = cam[0];
    double cam_y = cam[1];

    double x_position = homography_matrix[basestation_index][0][0] * cam_x + homography_matrix[basestation_index][0][1] * cam_y + homography_matrix[basestation_index][0][2];
    double y_position = homography_matrix[basestation_index][1][0] * cam_x + homography_matrix[basestation_index][1][1] * cam_y + homography_matrix[basestation_index][1][2];
    double scale      = homography_matrix[basestation_index][2][0] * cam_x + homography_matrix[basestation_index][2][1] * cam_y + homography_matrix[basestation_index][2][2];

    coordinates[0] = x_position / scale;
    coordinates[1] = y_position / scale;
}

void db_lh2_store_homography(db_lh2_t *lh2, uint8_t basestation_index, float homography_matrix_from_packet[3][3]) {
    if (basestation_index >= LH2_BASESTATION_COUNT) {
        return;
    }
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            homography_matrix[basestation_index][i][j] = (double)homography_matrix_from_packet[i][j];
        }
    }

    lh2->lh2_calibration_complete[basestation_index] = true;
}
// clang-format on

//=========================== localization.c at swarmit a702520 ================

#define LH2_BASESTATION_COUNT_MAX LH2_REF_STATIONS
#define LH2_VALID_MM_LEN          LH2_REF_SELECT_RECT_LEN
#define LH2_LINES_MAX             8  ///< swarmit's (8U), signed so that -Wsign-compare passes the copied ring arithmetic

// clang-format off
typedef struct {
    db_lh2_t                lh2;
    double                  coordinates[2];
    position_2d_t           position;
} localization_data_t;

static __attribute__((aligned(4))) localization_data_t _localization_data = { 0 };
static uint32_t _station_mask = 0;
static uint32_t _valid_mm[LH2_BASESTATION_COUNT_MAX][LH2_VALID_MM_LEN] = { 0 };  ///< resolved, by slot
static float    _homographies[LH2_BASESTATION_COUNT_MAX][3][3] = { 0 };          ///< by slot, for the floor lines
static db_lh2_floor_line_t _lines[LH2_LINES_MAX];                                ///< ring of lines not yet drained
static uint8_t             _lines_head  = 0;                                     ///< next slot written
static uint8_t             _lines_count = 0;                                     ///< lines held

static int32_t _clamp_mm(double v) {
    return (v > INT32_MAX) ? INT32_MAX : ((v < INT32_MIN) ? INT32_MIN : (int32_t)v);
}

bool localization_init(float homographies[][3][3], uint32_t station_mask, const uint32_t valid_mm[][LH2_VALID_MM_LEN]) {
    station_mask &= (1U << LH2_BASESTATION_COUNT_MAX) - 1U;
    printf("LH2 station mask 0x%04X\n", station_mask);
    bool sensor_ok = localization_start();

    for (uint8_t lh_index = 0; lh_index < LH2_BASESTATION_COUNT_MAX; lh_index++) {
        if (((station_mask >> lh_index) & 1U) == 0) {
            continue;
        }
        lh2_select_rect_resolve(valid_mm[lh_index], _valid_mm[lh_index]);
        printf("LH%u x [%u, %u] y [%u, %u] mm, H:\n", lh_index, _valid_mm[lh_index][0], _valid_mm[lh_index][2], _valid_mm[lh_index][1], _valid_mm[lh_index][3]);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                printf("%f ", (double)homographies[lh_index][i][j]);
            }
            printf("\n");
        }
        db_lh2_store_homography(&_localization_data.lh2, lh_index, homographies[lh_index]);
        memcpy(_homographies[lh_index], homographies[lh_index], sizeof(_homographies[0]));
    }
    _station_mask = station_mask;
    return sensor_ok;
}

/// Copy and clear both sweeps of a station when both are fresh
static bool _take_pair(uint8_t lh_index, uint32_t *count1, uint32_t *count2) {
    db_lh2_t *lh2 = &_localization_data.lh2;
    if (lh2->data_ready[0][lh_index] != DB_LH2_PROCESSED_DATA_AVAILABLE || lh2->data_ready[1][lh_index] != DB_LH2_PROCESSED_DATA_AVAILABLE) {
        return false;
    }
    db_lh2_stop();
    bool fresh = lh2->data_ready[0][lh_index] == DB_LH2_PROCESSED_DATA_AVAILABLE && lh2->data_ready[1][lh_index] == DB_LH2_PROCESSED_DATA_AVAILABLE;
    if (fresh) {
        *count1                      = lh2->locations[0][lh_index].lfsr_counts;
        *count2                      = lh2->locations[1][lh_index].lfsr_counts;
        lh2->data_ready[0][lh_index] = DB_LH2_NO_NEW_DATA;
        lh2->data_ready[1][lh_index] = DB_LH2_NO_NEW_DATA;
    }
    db_lh2_start();
    return fresh;
}

_Static_assert(LH2_LINES_MAX % 2 == 0, "pairs never straddle the end of the ring");

/// Pairs go straight into the ring: the head is always even, and
/// db_lh2_sweep_lines() writes nothing when it fails
static void _lines_push(uint32_t count1, uint32_t count2, uint8_t lh_index) {
    if (db_lh2_sweep_lines(count1, count2, lh_index, (const float(*)[3])_homographies[lh_index], &_lines[_lines_head]) != 2) {
        return;
    }
    _lines_head  = (uint8_t)((_lines_head + 2) % LH2_LINES_MAX);
    _lines_count = (_lines_count + 2 > LH2_LINES_MAX) ? LH2_LINES_MAX : (uint8_t)(_lines_count + 2);
}

uint8_t localization_get_lines(db_lh2_floor_line_t *out, uint8_t max) {
    uint8_t n     = (max < _lines_count) ? max : _lines_count;
    uint8_t first = (uint8_t)((_lines_head + LH2_LINES_MAX - _lines_count) % LH2_LINES_MAX);
    for (uint8_t i = 0; i < n; i++) {
        out[i] = _lines[(first + i) % LH2_LINES_MAX];
    }
    _lines_count -= n;
    return n;
}

bool localization_get_position(position_2d_t *position) {
    if (_station_mask == 0) {
        return false;
    }

    lh2_select_best_t best;
    lh2_select_reset(&best);
    for (uint8_t lh_index = 0; lh_index < LH2_BASESTATION_COUNT_MAX; lh_index++) {
        // Every fresh pair is taken, whatever its station, so none is used
        // stale by a later call; the decoder stops only for the copy
        uint32_t count1, count2;
        if (!_take_pair(lh_index, &count1, &count2)) {
            continue;
        }
        // A station without a homography would solve to NaN, or worse
        if (((_station_mask >> lh_index) & 1U) == 0) {
            continue;
        }
        db_lh2_calculate_position(count1, count2, lh_index, _localization_data.coordinates);
        double x = _localization_data.coordinates[0];
        double y = _localization_data.coordinates[1];
        // Reject a non-finite solve before the (long) casts below, which are
        // undefined for NaN and inf
        if (!isfinite(x) || !isfinite(y)) {
            printf("LH%u solve not finite\n", lh_index);
            continue;
        }
        if (!lh2_select_rect_contains(_valid_mm[lh_index], x, y)) {
            printf("LH%u (%ld,%ld) outside\n", lh_index, (long)_clamp_mm(x), (long)_clamp_mm(y));
            continue;
        }
        lh2_select_offer(&best, lh_index, x, y, _valid_mm[lh_index]);
        _lines_push(count1, count2, lh_index);
    }
    if (!best.found) {
        return false;
    }

    // Inside its station's rectangle, so within uint32
    _localization_data.position.x = (uint32_t)best.x;
    _localization_data.position.y = (uint32_t)best.y;
    position->x                   = _localization_data.position.x;
    position->y                   = _localization_data.position.y;
    printf("Position (%u,%u) from LH%u\n", position->x, position->y, best.station);
    return true;
}

uint8_t localization_get_raw_counts(lh2_raw_sample_t *out, uint8_t max) {
    uint8_t n = 0;
    db_lh2_stop();
    for (uint8_t lh_index = 0; lh_index < LH2_BASESTATION_COUNT && n < max; lh_index++) {
        if (_localization_data.lh2.data_ready[0][lh_index] == DB_LH2_PROCESSED_DATA_AVAILABLE && _localization_data.lh2.data_ready[1][lh_index] == DB_LH2_PROCESSED_DATA_AVAILABLE) {
            out[n].lh_index = lh_index;
            out[n].count1   = _localization_data.lh2.locations[0][lh_index].lfsr_counts;
            out[n].count2   = _localization_data.lh2.locations[1][lh_index].lfsr_counts;
            _localization_data.lh2.data_ready[0][lh_index] = DB_LH2_NO_NEW_DATA;
            _localization_data.lh2.data_ready[1][lh_index] = DB_LH2_NO_NEW_DATA;
            n++;
        }
    }
    db_lh2_start();
    return n;
}
// clang-format on

//=========================== hooks ============================================

#undef lh2_select_reset
#undef lh2_select_offer

static void _hook_reset(lh2_select_best_t *best) {
    lh2_select_reset(best);
    memset(&_best_seen, 0, sizeof(_best_seen));
}

static bool _hook_offer(lh2_select_best_t *best, uint8_t station, double x, double y, const uint32_t rect[LH2_SELECT_RECT_LEN]) {
    bool kept  = lh2_select_offer(best, station, x, y, rect);
    _best_seen = *best;
    return kept;
}

//=========================== public ===========================================

lh2_ref_t *lh2_ref_select_slots(void) {
    return &_localization_data.lh2;
}

lh2_ref_select_best_t lh2_ref_select_best(void) {
    lh2_ref_select_best_t out = { .found = _best_seen.found, .station = _best_seen.station, .x = _best_seen.x, .y = _best_seen.y, .d2 = _best_seen.d2 };
    return out;
}

void lh2_ref_select_clear(void) {
    memset(&_localization_data, 0, sizeof(_localization_data));
    memset(homography_matrix, 0, sizeof(homography_matrix));
    memset(_valid_mm, 0, sizeof(_valid_mm));
    memset(_homographies, 0, sizeof(_homographies));
    memset(_lines, 0, sizeof(_lines));
    memset(&_best_seen, 0, sizeof(_best_seen));
    _station_mask = 0;
    _lines_head   = 0;
    _lines_count  = 0;
}

void lh2_ref_select_rect_resolve(const uint32_t stored[LH2_REF_SELECT_RECT_LEN], uint32_t out[LH2_REF_SELECT_RECT_LEN]) {
    lh2_select_rect_resolve(stored, out);
}

bool lh2_ref_select_offer_solve(lh2_ref_select_best_t *best, uint8_t station, double x, double y, const uint32_t rect[LH2_REF_SELECT_RECT_LEN]) {
    lh2_select_best_t b    = { .found = best->found, .station = best->station, .x = best->x, .y = best->y, .d2 = best->d2 };
    bool              kept = lh2_select_offer(&b, station, x, y, rect);
    best->found            = b.found;
    best->station          = b.station;
    best->x                = b.x;
    best->y                = b.y;
    best->d2               = b.d2;
    return kept;
}
