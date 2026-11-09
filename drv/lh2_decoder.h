#ifndef __LH2_DECODER_H
#define __LH2_DECODER_H

/**
 * @defgroup    drv_lh2_decoder     LH2 decoder
 * @ingroup     drv
 * @brief       From one SPI capture of the TS4231 data line to a sweep's LFSR count
 *
 * A capture is 64 bytes of the data line sampled at 32 MHz, MSB first, from
 * the envelope's falling edge, plus the time it was stamped with. The decoder
 * demodulates its biphase-mark code, finds which of the 32 LFSR polynomials
 * the bits belong to (two per station), searches the LFSR index of the bits
 * (the count, the rotor angle in units of 8 ticks of 48 MHz) and stores it in
 * one of the station's two sweep slots, chosen by timestamp. A pair is both
 * slots of a station, the two counts a solve needs (%lh2_geometry.h).
 *
 * No hardware calls and no module state: the caller owns the decoder and
 * passes time in, as microseconds that wrap at 2^32 and are only compared by
 * unsigned subtraction. The capture queue is here too, so that its push and
 * pop are host-tested; the capture layer (bsp/%lh2.h) owns the one the
 * interrupt fills, and masks the interrupt around the pop.
 *
 * @{
 * @file
 * @copyright Inria, 2022-present
 * @}
 */

#include <stdbool.h>
#include <stdint.h>

//=========================== defines ==========================================

#define DB_LH2_STATIONS          (16U)          ///< LH2 channels, indexed by lh_index
#define DB_LH2_POLYNOMIALS       (32U)          ///< two per station: polynomial >> 1 is the station
#define DB_LH2_SWEEPS            (2U)           ///< sweep slots per station
#define DB_LH2_CAPTURE_BYTES     (64U)          ///< 16 us of the data line at 32 MHz, MSB first
#define DB_LH2_CHECKPOINTS       (64U)          ///< static checkpoints per polynomial
#define DB_LH2_QUEUE_LEN         (10U)          ///< captures the queue holds before it overwrites the oldest
#define DB_LH2_POLYNOMIAL_NONE   (0xFFU)        ///< no polynomial matched, or an empty slot
#define DB_LH2_INDEX_NONE        (0xFFFFFFFFU)  ///< the LFSR search found no index
#define DB_LH2_STALE_US          (2000000U)     ///< a slot older than this is emptied when its station is next decoded
#define DB_LH2_SLOT_TOLERANCE_US (1000U)        ///< a capture joins a filled slot within this of its period grid

/// Defined: the motion-capture check rejects captures that look like a
/// Qualisys strobe, and the polynomial match tolerates no bit error
#define LH2_MOCAP_FILTER 1

/// One capture of the TS4231 data line
typedef struct {
    uint8_t  spi[DB_LH2_CAPTURE_BYTES];  ///< from the envelope's falling edge
    uint32_t t_us;                       ///< the time the capture was stamped with, wraps at 2^32
} db_lh2_capture_t;

/// What one capture did to the decoder
typedef enum {
    DB_LH2_DECODE_STORED,           ///< its count is in a sweep slot, ready
    DB_LH2_DECODE_MOCAP,            ///< rejected as motion-capture light; nothing changed
    DB_LH2_DECODE_NO_POLYNOMIAL,    ///< no polynomial matched; nothing changed
    DB_LH2_DECODE_ZERO_BITS,        ///< the LFSR bits are all zero; the slot it chose is cleared
    DB_LH2_DECODE_NO_INDEX,         ///< the LFSR search failed; the slot it chose is cleared
    DB_LH2_DECODE_PAST_REVOLUTION,  ///< the count is past one revolution; the slot it chose is cleared
    DB_LH2_DECODE_EQUAL_COUNTS,     ///< the count equals the other sweep's; both slots cleared
    DB_LH2_DECODE_RESULTS,          ///< number of results, for the counters
} db_lh2_decode_result_t;

/// One sweep slot of one station
typedef struct {
    uint32_t count;       ///< LFSR count of the last capture stored here
    uint32_t t_us;        ///< its capture's stamp; 0 marks the slot empty for the slot choice
    uint8_t  polynomial;  ///< its polynomial, of this slot's station; DB_LH2_POLYNOMIAL_NONE before the first
    bool     ready;       ///< a count stored and not yet taken
} db_lh2_slot_t;

/// Both sweeps of one station, as db_lh2_decoder_take_pair() hands them over
typedef struct {
    uint8_t  station;   ///< lh_index 0..15
    uint32_t count[2];  ///< slot 0 and slot 1
    uint32_t t_us[2];   ///< their capture stamps
} db_lh2_pair_t;

/// The decoder, owned by the caller
typedef struct {
    db_lh2_slot_t slot[DB_LH2_SWEEPS][DB_LH2_STATIONS];                 ///< by sweep, then station
    uint32_t      checkpoint_bits[DB_LH2_POLYNOMIALS][DB_LH2_SWEEPS];   ///< LFSR bits of the last count found, per polynomial and slot
    uint32_t      checkpoint_count[DB_LH2_POLYNOMIALS][DB_LH2_SWEEPS];  ///< and that count, before the bit offset
    uint32_t      results[DB_LH2_DECODE_RESULTS];                       ///< captures fed, by result
} db_lh2_decoder_t;

/// Captures waiting to be decoded, oldest first
typedef struct {
    db_lh2_capture_t capture[DB_LH2_QUEUE_LEN];
    uint8_t          write;    ///< next slot pushed
    uint8_t          read;     ///< next slot popped
    uint8_t          count;    ///< captures held
    uint32_t         dropped;  ///< captures overwritten before they were popped
} db_lh2_queue_t;

//=========================== variables ========================================

/// LFSR taps of each polynomial, 17 bits
extern const uint32_t db_lh2_polynomials[DB_LH2_POLYNOMIALS];

/// Static checkpoints of the LFSR search: entry i of a polynomial is the
/// state whose top 6 bits are i, packed as index << 11 | (state & 0x7FF)
extern const uint32_t db_lh2_checkpoints[DB_LH2_POLYNOMIALS][DB_LH2_CHECKPOINTS];

//=========================== prototypes =======================================

/// Empty slots and no dynamic checkpoint
void db_lh2_decoder_init(db_lh2_decoder_t *dec);

/// Empty every slot; the dynamic checkpoints and the counters stay
void db_lh2_decoder_reset(db_lh2_decoder_t *dec);

/**
 * @brief   Decode one capture into its station's sweep slots
 *
 * @param[in,out]  dec     the decoder
 * @param[in]      cap     the capture
 * @param[in]      now_us  the time now, on the clock of cap->t_us; sets which slots are stale
 *
 * @return  what the capture did; dec->results counts it
 */
db_lh2_decode_result_t db_lh2_decoder_feed(db_lh2_decoder_t *dec, const db_lh2_capture_t *cap, uint32_t now_us);

/// Whether both slots of a station are ready
bool db_lh2_decoder_pair_ready(const db_lh2_decoder_t *dec, uint8_t station);

/**
 * @brief   Take both sweeps of a station when both are ready
 *
 * @param[in,out]  dec      the decoder
 * @param[in]      station  lh_index 0..15
 * @param[out]     out      both counts and their stamps
 *
 * @return  false, and nothing written or changed, unless both are ready; the slots are no longer ready after
 */
bool db_lh2_decoder_take_pair(db_lh2_decoder_t *dec, uint8_t station, db_lh2_pair_t *out);

/**
 * @brief   Demodulate a capture into 64 bits, the first received the MSB
 *
 * @param[in]   spi     the capture's bytes
 *
 * @return  the bits; a chip the demodulation could not settle reads 0
 */
uint64_t db_lh2_demodulate(const uint8_t spi[DB_LH2_CAPTURE_BYTES]);

/**
 * @brief   Find the polynomial a demodulated word belongs to
 *
 * @param[in]   bits        db_lh2_demodulate()'s word
 * @param[out]  bit_offset  bits skipped before the 17-bit seed the match starts from, 8 or 9
 *
 * @return  0..31, or DB_LH2_POLYNOMIAL_NONE
 */
uint8_t db_lh2_polynomial(uint64_t bits, int8_t *bit_offset);

/**
 * @brief   LFSR index of a 17-bit state, from the static and the decoder's dynamic checkpoints
 *
 * @param[in]   dec         the decoder, for its dynamic checkpoints
 * @param[in]   polynomial  0..31
 * @param[in]   bits        the 17-bit state
 *
 * @return  the index, or DB_LH2_INDEX_NONE
 */
uint32_t db_lh2_lfsr_index(const db_lh2_decoder_t *dec, uint8_t polynomial, uint32_t bits);

/// Whether a capture looks like a motion-capture strobe: 3 equal bytes of all
/// ones or all zeros in its first half
bool db_lh2_mocap_interference(const uint8_t spi[DB_LH2_CAPTURE_BYTES]);

/// Empty the queue; the drop counter stays
void db_lh2_queue_clear(db_lh2_queue_t *queue);

/// Add a capture, overwriting the oldest and counting it dropped when full
void db_lh2_queue_push(db_lh2_queue_t *queue, const uint8_t spi[DB_LH2_CAPTURE_BYTES], uint32_t t_us);

/// Remove the oldest capture; false when empty
bool db_lh2_queue_pop(db_lh2_queue_t *queue, db_lh2_capture_t *out);

#endif
