#ifndef __LH2_REFERENCE_SELECT_H
#define __LH2_REFERENCE_SELECT_H

/**
 * @file
 * @brief  Frozen copy of the LH2 station choice at swarmit a702520
 *
 * One instance, like the bootloader: the calibration, the slots and the ring
 * are module state.
 *
 * @copyright Inria, 2026
 */

#include <stdbool.h>
#include <stdint.h>

#include "lh2_geometry.h"
#include "lh2_reference.h"

#define LH2_REF_SELECT_COMMIT    "a702520"
#define LH2_REF_SELECT_RECT_LEN  (4U)
#define LH2_REF_SELECT_LINES_MAX (8U)

/// swarmit's position_2d_t
typedef struct __attribute__((packed)) {
    uint32_t x;
    uint32_t y;
} lh2_ref_position_t;

/// swarmit's lh2_raw_sample_t
typedef struct {
    uint32_t count1;
    uint32_t count2;
    uint8_t  lh_index;
    uint8_t  _pad[3];
} lh2_ref_raw_sample_t;

/// The solve the last fix kept, as lh2_select_best_t held it
typedef struct {
    bool    found;
    uint8_t station;
    double  x;
    double  y;
    double  d2;
} lh2_ref_select_best_t;

/// localization_init() past the sensor start
bool lh2_ref_select_init(float homographies[][3][3], uint32_t station_mask, const uint32_t valid_mm[][LH2_REF_SELECT_RECT_LEN]);

/// localization_get_position()
bool lh2_ref_select_get_position(lh2_ref_position_t *position);

/// localization_get_lines()
uint8_t lh2_ref_select_get_lines(db_lh2_floor_line_t *out, uint8_t max);

/// localization_get_raw_counts()
uint8_t lh2_ref_select_get_raw_counts(lh2_ref_raw_sample_t *out, uint8_t max);

/// The BSP slots the pair take and the drain read; the test fills them
lh2_ref_t *lh2_ref_select_slots(void);

/// The solve the last lh2_ref_select_get_position() kept
lh2_ref_select_best_t lh2_ref_select_best(void);

/// Every piece of module state back to boot
void lh2_ref_select_clear(void);

/// lh2_select_rect_resolve()
void lh2_ref_select_rect_resolve(const uint32_t stored[LH2_REF_SELECT_RECT_LEN], uint32_t out[LH2_REF_SELECT_RECT_LEN]);

/// lh2_select_offer() on a best kept outside the module
bool lh2_ref_select_offer_solve(lh2_ref_select_best_t *best, uint8_t station, double x, double y, const uint32_t rect[LH2_REF_SELECT_RECT_LEN]);

#endif
