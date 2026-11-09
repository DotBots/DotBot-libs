#ifndef __LH2_REFERENCE_H
#define __LH2_REFERENCE_H

/**
 * @file
 * @brief  Frozen copy of the LH2 decode path at DotBot-libs 228ce9a
 *
 * All of bsp/nrf/lh2_decoder.c and, from bsp/nrf/lh2_default.c, the process
 * function, the sweep-slot rule and the checkpoint update, as they stand at
 * 228ce9a (unchanged through 61e762c). Three touch points are hooks: the
 * capture is passed in instead of popped from the ring, the timer read
 * returns now_us, and the interrupt masking is gone. The copy is the
 * definition of today's behaviour: it is never edited to follow the BSP.
 *
 * One instance, like the BSP: the dynamic checkpoints are module state.
 *
 * @copyright Inria, 2026
 */

#include <stdbool.h>
#include <stdint.h>

//=========================== defines ==========================================

#define LH2_REF_COMMIT           "228ce9a"
#define LH2_REF_STATIONS         (16U)
#define LH2_REF_POLYNOMIALS      (32U)
#define LH2_REF_SWEEPS           (2U)
#define LH2_REF_CHECKPOINTS      (64U)  ///< static checkpoints per polynomial
#define LH2_REF_CAPTURE_BYTES    (64U)
#define LH2_REF_POLYNOMIAL_ERROR (0xFFU)
#define LH2_REF_INDEX_ERROR      (0xFFFFFFFFU)

/// db_lh2_data_ready_state_t
typedef enum {
    LH2_REF_NO_NEW_DATA,
    LH2_REF_RAW_DATA_AVAILABLE,
    LH2_REF_PROCESSED_DATA_AVAILABLE,
} lh2_ref_data_ready_t;

/// db_lh2_location_t
typedef struct __attribute__((packed)) {
    uint32_t lfsr_counts;
    uint32_t selected_polynomial;
} lh2_ref_location_t;

/// db_lh2_t, field for field
typedef struct {
    lh2_ref_location_t   locations[LH2_REF_SWEEPS][LH2_REF_STATIONS];
    uint32_t             timestamps[LH2_REF_SWEEPS][LH2_REF_STATIONS];
    lh2_ref_data_ready_t data_ready[LH2_REF_SWEEPS][LH2_REF_STATIONS];
    uint8_t             *spi_ring_buffer_count_ptr;
    bool                 lh2_calibration_complete[LH2_REF_STATIONS];
} lh2_ref_t;

/// _lfsr_checkpoint_t: the dynamic checkpoints
typedef struct {
    uint32_t bits[LH2_REF_POLYNOMIALS][LH2_REF_SWEEPS];
    uint32_t count[LH2_REF_POLYNOMIALS][LH2_REF_SWEEPS];
} lh2_ref_checkpoint_t;

/// The return db_lh2_process_location() took
typedef enum {
    LH2_REF_STORED,
    LH2_REF_MOCAP,
    LH2_REF_NO_POLYNOMIAL,
    LH2_REF_ZERO_BITS,
    LH2_REF_NO_INDEX,
    LH2_REF_PAST_REVOLUTION,
    LH2_REF_EQUAL_COUNTS,
} lh2_ref_result_t;

//=========================== prototypes =======================================

/// The module as at boot, and what db_lh2_init() does to lh2 past the hardware
void lh2_ref_init(lh2_ref_t *lh2);

/// db_lh2_reset(); the dynamic checkpoints stay, as on the robot
void lh2_ref_reset(lh2_ref_t *lh2);

/**
 * @brief   One db_lh2_process_location() call on one capture
 *
 * @param[in]   spi     the 64 bytes the SPIM captured
 * @param[in]   t_us    the time the SPIM handler stamped the capture with
 * @param[in]   now_us  what the slot rule's timer read returns
 */
lh2_ref_result_t lh2_ref_feed(lh2_ref_t *lh2, const uint8_t spi[LH2_REF_CAPTURE_BYTES], uint32_t t_us, uint32_t now_us);

/// The module's dynamic checkpoints
const lh2_ref_checkpoint_t *lh2_ref_checkpoints(void);

/// _demodulate_light() on a capture, padded to the 128 bytes it reads
uint64_t lh2_ref_demodulate(const uint8_t spi[LH2_REF_CAPTURE_BYTES]);

/// _determine_polynomial()
uint8_t lh2_ref_determine_polynomial(uint64_t bits, int8_t *start_val);

/// _lfsr_index_search() against the given dynamic checkpoints
uint32_t lh2_ref_lfsr_index_search(lh2_ref_checkpoint_t *checkpoints, uint8_t polynomial, uint32_t bits);

/// _check_mocap_interference() over the 64 bytes, as the process function calls it
bool lh2_ref_mocap_interference(const uint8_t spi[LH2_REF_CAPTURE_BYTES]);

/// _polynomials[index]
uint32_t lh2_ref_polynomial(uint8_t index);

/// _lfsr_index_table[polynomial][slot]: the LFSR index of a static checkpoint
uint32_t lh2_ref_static_checkpoint(uint8_t polynomial, uint8_t slot);

/// _lfsr_hash_table[polynomial][slot]: the LFSR state at that index
uint32_t lh2_ref_static_checkpoint_state(uint8_t polynomial, uint8_t slot);

#endif
