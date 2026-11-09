/**
 * @file
 * @ingroup drv_lh2_decoder
 *
 * @brief  LH2 demodulation, polynomial match, LFSR search and sweep slots
 *
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022-present
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lh2_decoder.h"
#include "lh2_geometry.h"

//=========================== defines ==========================================

_Static_assert(DB_LH2_STATIONS == DB_LH2_GEOMETRY_STATIONS, "lh2_geometry carries a period per station");

#define FUZZY_CHIP                             0xFF  ///< a chip the thresholds could not settle
#define POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD 0     ///< bit errors the polynomial match tolerates
#define HASH_TABLE_BITS                        6     ///< state bits that index the static checkpoints
#define HASH_TABLE_MASK                        ((1 << HASH_TABLE_BITS) - 1)
#define CHECKPOINT_STATE_BITS                  11  ///< state bits a packed checkpoint keeps, below the index bits
#define CHECKPOINT_STATE_MASK                  ((1U << CHECKPOINT_STATE_BITS) - 1U)
#define DEMODULATION_BYTES                     128  ///< the demodulator reads this many bytes, the capture then zeros

typedef uint32_t lfsr_17bits_t;

/// Which of a station's two slots are filled, slot 1 in bit 1
typedef enum {
    LH2_SWEEP_BOTH_SLOTS_EMPTY,
    LH2_SWEEP_SECOND_SLOT_EMPTY,
    LH2_SWEEP_FIRST_SLOT_EMPTY,
    LH2_SWEEP_BOTH_SLOTS_FULL,
} db_lh2_sweep_slot_state_t;

//=========================== private ==========================================

/// Zero-crossing demodulation of the biphase-mark code, over DEMODULATION_BYTES
static uint64_t _demodulate_light(uint8_t *sample_buffer) {
    // FIXME: an initial "1" can be thrown away without counting it in the bit offset, which puts the LFSR count 1 off
    uint8_t chip_index;
    uint8_t zccs_1[256];  // indexed by the uint8_t chip_index, which counts past 128 on a busy capture
    uint8_t chips1[128];
    uint8_t temp_byte_N;
    uint8_t temp_byte_M;

    // initialize loop variables
    int      jj = 0;
    int      kk = 0;
    uint64_t gg = 0;

    // initialize temporary "ones counter" variable that counts consecutive ones
    int ones_counter = 0;

    // initialize result:
    uint64_t chipsH1 = 0;

    // FIND ZERO CROSSINGS
    // An entry no crossing reaches keeps this count, which thresholds to a
    // zero chip, so the tail past the last crossing is deterministic.
    memset(zccs_1, 0xFF, sizeof(zccs_1));
    chip_index         = 0;
    zccs_1[chip_index] = 0x01;

    // for loop over bytes of the SPI buffer (jj), nested with a for loop over bits in each byte (ii)
    for (jj = 0; jj < 128; jj++) {
        // edge case - check if last bit (LSB) of previous byte is the same as first bit (MSB) of current byte
        // if it is not, increment chip_index and reset count
        if (jj != 0) {
            temp_byte_M = (sample_buffer[jj - 1]) & (0x01);   // previous byte's LSB
            temp_byte_N = (sample_buffer[jj] >> 7) & (0x01);  // current byte's MSB
            if (temp_byte_M != temp_byte_N) {
                chip_index++;
                zccs_1[chip_index] = 1;
            } else {
                zccs_1[chip_index] += 1;
            }
        }
        // detect intra-byte transitions with a single XOR: bit k is set when bit(k+1) != bit(k)
        uint8_t transitions = sample_buffer[jj] ^ (sample_buffer[jj] >> 1);
        for (int bit = 6; bit >= 0; bit--) {
            if ((transitions >> bit) & 1) {
                chip_index++;
                zccs_1[chip_index] = 1;
            } else {
                zccs_1[chip_index]++;
            }
        }
    }

    // threshold the zero crossings into: likely one chip, likely two zero chips, or fuzzy
    for (jj = 0; jj < 128; jj++) {
        if (zccs_1[jj] >= 5) {
            chips1[jj] = 0;  // it's a very likely zero
        } else if (zccs_1[jj] <= 3) {
            chips1[jj] = 1;  // it's a very likely one
        } else {
            chips1[jj] = FUZZY_CHIP;  // fuzzy
        }
    }

    // DEMODULATION:
    // basic principles, in descending order of importance:
    //  1) an odd number of ones in a row is not allowed - this must be avoided at all costs
    //  2) finding a solution to #1 given a set of data is quite cumbersome without certain assumptions
    //    a) a fuzzy before an odd run of 1s is almost always a 1
    //    b) a fuzzy between two even runs of 1s is almost always a 0
    //    c) a fuzzy after an even run of 1s is usually a a 0
    //  3) a detected 1 is rarely wrong, but detected 0s can be, this is especially common in low-SNR readings
    //    exception: if the first bit is a 1 it is NOT reliable because the capture is asynchronous
    //  4) this is not perfect, but the earlier the chip, the more likely that it is correct. Polynomials can be used to fix bit errors later in the reading
    // known bugs/issues:
    //  1) if there are many ones at the very beginning of the reading, the algorithm will mess it up
    //  2) in some instances, the count value will be off by approximately 5, the origin of this bug is unknown at the moment

    // reset variables:
    kk           = 0;
    ones_counter = 0;
    jj           = 0;
    for (jj = 0; jj < 128;) {
        if (chips1[jj] == 0x00) {  // zero, keep going, reset state
            jj++;
            ones_counter = 0;
            if (jj >= 128) {
                break;
            }
        }
        if (chips1[jj] == 0x01) {  // one, keep going, keep track of the # of ones
            if (jj == 0) {         // edge case - first chip = 1 is unreliable, do not increment 1s counter
                jj++;
            } else {
                jj           = jj + 1;
                ones_counter = ones_counter + 1;
            }
            if (jj >= 128) {
                break;
            }
        }

        if ((jj == 127) & (chips1[jj] == FUZZY_CHIP)) {
            chips1[jj] = 0x00;
        } else if ((chips1[jj] == FUZZY_CHIP) & (ones_counter == 0)) {  // fuzz after a zero
            if (chips1[jj + 1] == 0) {                                  // zero then fuzz then zero -> fuzz is a zero
                jj++;
                chips1[jj - 1] = 0;
            } else if (chips1[jj + 1] == FUZZY_CHIP) {  // zero then fuzz then fuzz -> just move on, you're probably screwed
                jj += 2;
            } else if (chips1[jj + 1] == 1) {  // zero then fuzz then one -> investigate
                kk           = 1;
                ones_counter = 0;
                while (jj + kk < 128 && chips1[jj + kk] == 1) {
                    ones_counter++;
                    kk++;
                }
                if (ones_counter % 2 == 1) {  // fuzz -> odd ones, the fuzz is a 1
                    jj++;
                    chips1[jj - 1] = 1;
                    ones_counter   = 1;
                } else if (ones_counter % 2 == 0) {  // fuzz -> even ones, move on for now, it's indeterminate
                    jj++;
                    ones_counter = 0;  // temporarily treat as a 0 for counting purposes
                } else {               // catch statement
                    jj++;
                }
            }
        } else if ((chips1[jj] == FUZZY_CHIP) & (ones_counter != 0)) {  // ones then fuzz
            if ((ones_counter % 2 == 0) & (chips1[jj + 1] == 0)) {      // even ones then fuzz then zero, fuzz is a zero
                jj++;
                chips1[jj - 1] = 0;
                ones_counter   = 0;
            }
            if ((ones_counter % 2 == 0) && (jj + 1 < 128) && (chips1[jj + 1] != 0)) {  // even ones then fuzz then not zero - investigate
                if (chips1[jj + 1] == 1) {                                             // subsequent bit is a 1
                    kk = 1;
                    while (jj + kk < 128 && chips1[jj + kk] == 1) {
                        ones_counter++;
                        kk++;
                    }
                    if (ones_counter % 2 == 1) {  // indicates an odd # of 1s, so the fuzzy has to be a 1
                        jj++;
                        chips1[jj - 1] = 1;
                        ones_counter   = 1;              // not actually 1, but it's ok for modulo purposes
                    } else if (ones_counter % 2 == 0) {  // even ones -> fuzz -> even ones, indeterminate
                        jj++;
                        ones_counter = 0;
                    }
                } else if (chips1[jj + 1] == FUZZY_CHIP) {  // subsequent bit is a fuzzy - skip for now...
                    jj++;
                }
            } else if ((ones_counter % 2 == 1) && (chips1[jj + 1] == FUZZY_CHIP)) {  // odd ones then fuzz then fuzz, fuzz is 1 then 0
                jj += 2;
                chips1[jj - 1] = 0;
                chips1[jj - 2] = 1;
                ones_counter   = 0;
            } else if ((ones_counter % 2 == 1) && (chips1[jj + 1] != 0)) {  // odd ones then fuzz then not zero - the fuzzy has to be a 1
                jj++;
                ones_counter++;
                chips1[jj - 1] = 1;
            } else {  // catch statement
                jj++;
            }
        }
    }
    // finish up demodulation, pick off straggling fuzzies and odd runs of 1s
    ones_counter = 0;
    for (jj = 0; jj < 128;) {
        if (chips1[jj] == 0x00) {                   // zero, keep going, reset state
            if (ones_counter % 2 == 1) {            // implies an odd # of 1s
                chips1[jj - ones_counter - 1] = 1;  // change the bit before the run of 1s to a 1 to make it even
            }
            jj++;
            ones_counter = 0;
        } else if (chips1[jj] == 0x01) {  // one, keep going, keep track of the # of ones
            if (jj == 0) {                // edge case - first chip = 1 is unreliable, do not increment 1s counter
                jj++;
            } else {
                jj           = jj + 1;
                ones_counter = ones_counter + 1;
            }
        } else if (chips1[jj] == FUZZY_CHIP) {
            // A fuzz after zeros is left alone: making it a 1 when a 1 follows
            // is a rare case that broke otherwise clean captures
            if ((ones_counter != 0) & (ones_counter % 2 == 0)) {  // fuzz after even ones - at this point this is almost always a 0
                jj++;
                chips1[jj - 1] = 0;
                ones_counter   = 0;
            } else if (ones_counter % 2 == 1) {  // fuzz after odd ones - exceedingly uncommon at this point, make it a 1
                jj++;
                chips1[jj - 1] = 1;
                ones_counter++;
            } else {  // catch statement
                jj++;
            }
        } else {  // catch statement
            jj++;
        }
    }

    // pack the chips into 64 bits, a 1 taking two chips and a 0 or a fuzz one; a leading 1 or fuzz is skipped
    chip_index = 0;
    chipsH1    = 0;
    gg         = 0;
    while (gg < 64) {
        if (chip_index > 127) {
            break;
        }
        if ((chip_index == 0) & (chips1[chip_index] == 0x01)) {  // first bit is a 1 - ignore it
            chip_index = chip_index + 1;
        } else if ((chip_index == 0) & (chips1[chip_index] == FUZZY_CHIP)) {  // first bit is fuzzy - ignore it
            chip_index = chip_index + 1;
        } else if (gg == 63) {  // load the final bit
            if (chips1[chip_index] == 0) {
                chipsH1 &= 0xFFFFFFFFFFFFFFFE;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == FUZZY_CHIP) {
                chipsH1 &= 0xFFFFFFFFFFFFFFFE;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == 0x01) {
                chipsH1 |= 0x0000000000000001;
                gg         = gg + 1;
                chip_index = chip_index + 2;
            }
        } else {  // load the bit in
            if (chips1[chip_index] == 0) {
                chipsH1    = chipsH1 << 1;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == FUZZY_CHIP) {
                chipsH1    = chipsH1 << 1;
                gg         = gg + 1;
                chip_index = chip_index + 1;
            } else if (chips1[chip_index] == 0x01) {
                chipsH1 |= 0x0000000000000001;
                chipsH1    = chipsH1 << 1;
                gg         = gg + 1;
                chip_index = chip_index + 2;
            }
        }
    }
    return chipsH1;
}

/// The 17-bit seed bits and the numbits an LFSR with taps poly runs on from it, newest the LSB
static uint64_t _poly_check(uint32_t poly, uint32_t bits, uint8_t numbits) {
    uint64_t bits_out      = 0;
    uint8_t  shift_counter = 1;
    uint8_t  b1            = 0;
    uint32_t buffer        = bits;
    poly &= 0x00001FFFF;
    bits_out |= buffer;              // initialize 17 LSBs of result
    bits_out &= 0x00000000FFFFFFFF;  // mask because I didn't want to re-cast the buffer

    while (shift_counter <= numbits) {
        bits_out = bits_out << 1;  // shift left (forward in time) by 1

        b1     = __builtin_popcount(buffer & poly) & 0x01;  // mask the buffer w/ the selected polynomial
        buffer = ((buffer << 1) | b1) & (0x0001FFFF);

        bits_out |= ((b1) & (0x01));  // put result of the XOR operation into the new bit
        shift_counter++;
    }
    return bits_out;
}

//=========================== public ===========================================

uint64_t db_lh2_demodulate(const uint8_t spi[DB_LH2_CAPTURE_BYTES]) {
    uint8_t samples[DEMODULATION_BYTES] = { 0 };
    memcpy(samples, spi, DB_LH2_CAPTURE_BYTES);
    return _demodulate_light(samples);
}

uint8_t db_lh2_polynomial(uint64_t chipsH1, int8_t *start_val) {
    // Handle the edgecase of a full zero input
    if (chipsH1 == 0x00) {
        return DB_LH2_POLYNOMIAL_NONE;
    }

    *start_val = 8;

    int32_t  bits_N_for_comp                    = 47 - *start_val;
    uint32_t bit_buffer1                        = (uint32_t)(((0xFFFF800000000000) & chipsH1) >> 47);
    uint64_t bits_from_poly[DB_LH2_POLYNOMIALS] = { 0 };
    uint8_t  weights[DB_LH2_POLYNOMIALS]        = { 0 };
    uint8_t  selected_poly                      = DB_LH2_POLYNOMIAL_NONE;  // initialize to error condition
    uint8_t  min_weight_idx                     = DB_LH2_POLYNOMIAL_NONE;
    uint64_t min_weight                         = DB_LH2_POLYNOMIAL_NONE;
    uint64_t bits_to_compare                    = 0;
    int32_t  threshold                          = POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD;

#if defined(LH2_MOCAP_FILTER)
    // tighten threshold if we expect MoCap interference
    threshold = 0;
#endif

    // A 17-bit seed from the word is run forward through every polynomial's
    // LFSR and compared with the bits that follow it. On a miss the seed moves
    // one bit later, or, already moved, back to bit 8 with 9 fewer bits compared.
    while (1) {
        bit_buffer1     = (uint32_t)(((0xFFFF800000000000 >> (*start_val)) & chipsH1) >> (64 - 17 - (*start_val)));
        bits_to_compare = (chipsH1 & (0xFFFFFFFFFFFFFFFF << (64 - 17 - (*start_val) - bits_N_for_comp)));
        // reset the minimum polynomial match found
        min_weight_idx = DB_LH2_POLYNOMIAL_NONE;
        min_weight     = DB_LH2_POLYNOMIAL_NONE;
        // Check against all the known polynomials
        for (uint8_t i = 0; i < DB_LH2_POLYNOMIALS; i++) {
            bits_from_poly[i] = (((_poly_check(db_lh2_polynomials[i], bit_buffer1, bits_N_for_comp)) << (64 - 17 - (*start_val) - bits_N_for_comp)) | (chipsH1 & (0xFFFFFFFFFFFFFFFF << (64 - (*start_val)))));
            weights[i]        = __builtin_popcountll(bits_from_poly[i] ^ bits_to_compare);
            // Keep track of the minimum weight value and which polinimial generated it.
            if (weights[i] < min_weight) {
                min_weight_idx = i;
                min_weight     = weights[i];
            }
        }

        // If you found a sufficiently good value, then return which polinomial generated it
        if (min_weight <= (uint64_t)threshold) {
            selected_poly = min_weight_idx;
            break;
            // match failed, try again removing bits from the end
        } else if (*start_val > 8) {
            *start_val      = 8;
            bits_N_for_comp = bits_N_for_comp - 9;
            if (threshold > 2) {
                threshold = threshold - 1;
            } else if (threshold == 2) {  // keep threshold at ones, but you're probably screwed with an unlucky bit error
                threshold = 2;
            }
        } else {
            *start_val = *start_val + 1;
            bits_N_for_comp -= 1;
        }

        // too few bits to reliably compare, give up
        if (bits_N_for_comp < 19) {
            selected_poly = DB_LH2_POLYNOMIAL_NONE;  // mark the poly as "wrong"
            break;
        }
    }
    return selected_poly;
}

uint32_t db_lh2_lfsr_index(const db_lh2_decoder_t *dec, uint8_t index, uint32_t bits) {

    lfsr_17bits_t bits_local  = bits & 0x0001FFFF;  // initialize buffer to initial bits, masked
    lfsr_17bits_t buffer_down = bits_local;
    lfsr_17bits_t buffer_up   = bits_local;

    uint32_t count_down  = 0;
    uint32_t count_up    = 0;
    int32_t  count_final = 0;  // There is a chance for overflow near the roll-over point of the lfsr sequence (sequence very high (>120k), hitting a very low check point (~20))
    uint32_t b17         = 0;
    uint32_t b1          = 0;
    uint32_t masked_buff = 0;
    uint8_t  hash_down   = 0;
    uint8_t  hash_up     = 0;
    bool     success     = false;  // True if the LFSR search produced a valid result

    // Copy the static checkpoints (flash) into RAM, which the loop reads faster
    uint32_t _lfsr_checkpoints_local[DB_LH2_CHECKPOINTS] = { 0 };
    uint32_t polynomials_local                           = db_lh2_polynomials[index];
    for (size_t i = 0; i < DB_LH2_CHECKPOINTS; i++) {
        _lfsr_checkpoints_local[i] = db_lh2_checkpoints[index][i];
    }

    // Start the iterative search
    while (buffer_up != 0x00 && buffer_down != 0x00) {  // Check that the count has not fallen into an invalid state

        //
        // CHECKPOINT CHECKING
        //

        // Check lfsr backward count against precomputed checkpoints; the
        // entry at the state's top bits holds that state's low bits
        hash_down = (buffer_down >> 11) & HASH_TABLE_MASK;
        if ((buffer_down & CHECKPOINT_STATE_MASK) == (_lfsr_checkpoints_local[hash_down] & CHECKPOINT_STATE_MASK)) {
            count_final = count_down + (_lfsr_checkpoints_local[hash_down] >> CHECKPOINT_STATE_BITS);
            success     = true;  // mark the success of the search
            break;
        }

        // Check lfsr forward count against precomputed checkpoints
        hash_up = (buffer_up >> 11) & HASH_TABLE_MASK;
        if ((buffer_up & CHECKPOINT_STATE_MASK) == (_lfsr_checkpoints_local[hash_up] & CHECKPOINT_STATE_MASK)) {
            count_final = (_lfsr_checkpoints_local[hash_up] >> CHECKPOINT_STATE_BITS) - count_up;
            success     = true;  // mark the success of the search
            break;
        }

        // Check the dynamical checkpoints, backward
        // Sweep 0
        if (buffer_down == dec->checkpoint_bits[index][0]) {
            count_final = count_down + dec->checkpoint_count[index][0];
            success     = true;  // mark the success of the search
            break;
        }
        // Sweep 1
        if (buffer_down == dec->checkpoint_bits[index][1]) {
            count_final = count_down + dec->checkpoint_count[index][1];
            success     = true;  // mark the success of the search
            break;
        }

        // Check the dynamical checkpoints, forward
        // Sweep 0
        if (buffer_up == dec->checkpoint_bits[index][0]) {
            count_final = dec->checkpoint_count[index][0] - count_up;
            success     = true;  // mark the success of the search
            break;
        }
        // Sweep 1
        if (buffer_up == dec->checkpoint_bits[index][1]) {
            count_final = dec->checkpoint_count[index][1] - count_up;
            success     = true;  // mark the success of the search
            break;
        }

        //
        // LSFR UPDATE
        //

        // LSFR backward update
        b17         = buffer_down & 0x00000001;                                                      // save the "newest" bit of the buffer
        buffer_down = (buffer_down & (0x0001FFFE)) >> 1;                                             // shift the buffer right, backwards in time
        masked_buff = (buffer_down) & (polynomials_local);                                           // mask the buffer w/ the selected polynomial
        buffer_down = buffer_down | (((__builtin_popcount(masked_buff) ^ b17) & 0x00000001) << 16);  // This weird line propagates the LSFR one bit into the past
        count_down++;

        // LSFR forward update
        b1        = __builtin_popcount(buffer_up & polynomials_local) & 0x01;  // mask the buffer w/ the selected polynomial
        buffer_up = ((buffer_up << 1) | b1) & (0x0001FFFF);
        count_up++;
    }

    // Return the found lfsr value
    if (success) {
        // Handle overflows
        if (count_final < 0) {
            // wrap around the number
            count_final += 131071;  // 2^17 -1 (the full lenght of the sequence)
        }
        // Turn result back to unsigned before returning
        return (uint32_t)count_final;
    } else {
        return DB_LH2_INDEX_NONE;
    }
}

bool db_lh2_mocap_interference(const uint8_t spi[DB_LH2_CAPTURE_BYTES]) {
    const uint8_t *arr  = spi;
    uint8_t        size = DB_LH2_CAPTURE_BYTES;

    // Qualysis Mocap cameras pulse IR light modulated with a regular square wave at 1Mhz.
    // At the 32Mhz speed we sample the SPI, that corresponds to an alternating 0xFF,0xFF,0xFF,0x00,0x00,0x00 pattern

    // Check only the bottom half of the array, that should be enough to catch an error.
    for (int i = 0; i < size / 2; i++) {
        // Check for 3 consecutive 0xFF
        if (arr[i] == 0xFF && arr[i + 1] == 0xFF && arr[i + 2] == 0xFF) {
            return true;  // Error for 3 consecutive 0xFF
        }

        // Check for 3 consecutive 0x00
        if (arr[i] == 0x00 && arr[i + 1] == 0x00 && arr[i + 2] == 0x00) {
            return true;  // Error for 3 consecutive 0x00
        }
    }
    return false;  // No error found
}

//=========================== sweep slots ======================================

/// The slot a station's new capture goes to: a filled slot whose capture is a
/// whole number of rotor periods away, within DB_LH2_SLOT_TOLERANCE_US, else an
/// empty one. Slots older than DB_LH2_STALE_US are emptied first.
static uint8_t _select_sweep(db_lh2_decoder_t *dec, uint8_t polynomial, uint32_t timestamp, uint32_t now) {
    uint8_t  basestation  = polynomial >> 1;
    uint16_t sweep_period = (uint16_t)db_lh2_period_us(basestation);

    for (size_t sweep = 0; sweep < 2; sweep++) {
        if (now - dec->slot[sweep][basestation].t_us > DB_LH2_STALE_US) {
            dec->slot[sweep][basestation].t_us  = 0;
            dec->slot[sweep][basestation].ready = false;
        }
    }

    uint8_t sweep_slot_state = ((dec->slot[1][basestation].t_us != 0) << 1) | (dec->slot[0][basestation].t_us != 0);
    uint8_t selected_sweep   = 0;

    switch (sweep_slot_state) {

        case LH2_SWEEP_BOTH_SLOTS_EMPTY:
        {
            selected_sweep = 0;
            break;
        }

        case LH2_SWEEP_FIRST_SLOT_EMPTY:
        {
            // distance to the filled slot's period grid
            uint32_t diff = (timestamp - dec->slot[1][basestation].t_us) % sweep_period;
            diff          = diff < sweep_period - diff ? diff : sweep_period - diff;

            if (diff < DB_LH2_SLOT_TOLERANCE_US) {
                selected_sweep = 1;
            } else {
                selected_sweep = 0;
            }
            break;
        }

        case LH2_SWEEP_SECOND_SLOT_EMPTY:
        {
            uint32_t diff = (timestamp - dec->slot[0][basestation].t_us) % sweep_period;
            diff          = diff < sweep_period - diff ? diff : sweep_period - diff;

            if (diff < DB_LH2_SLOT_TOLERANCE_US) {
                selected_sweep = 0;
            } else {
                selected_sweep = 1;
            }
            break;
        }

        case LH2_SWEEP_BOTH_SLOTS_FULL:
        {
            int64_t diff_0 = ((timestamp - dec->slot[0][basestation].t_us) % sweep_period);
            diff_0         = diff_0 < sweep_period - diff_0 ? diff_0 : sweep_period - diff_0;
            int64_t diff_1 = ((timestamp - dec->slot[1][basestation].t_us) % sweep_period);
            diff_1         = diff_1 < sweep_period - diff_1 ? diff_1 : sweep_period - diff_1;

            // the one nearer its period grid; slot 0 on a tie
            if (diff_0 <= diff_1) {
                selected_sweep = 0;
            } else {
                selected_sweep = 1;
            }
            break;
        }

        default:
        {
            selected_sweep = 0;
            break;
        }
    }

    return selected_sweep;
}

static db_lh2_decode_result_t _decode(db_lh2_decoder_t *dec, const db_lh2_capture_t *cap, uint32_t now_us) {
#if defined(LH2_MOCAP_FILTER)
    if (db_lh2_mocap_interference(cap->spi)) {
        return DB_LH2_DECODE_MOCAP;
    }
#endif

    uint64_t temp_bits_sweep = db_lh2_demodulate(cap->spi);

    int8_t  temp_bit_offset          = 0;
    uint8_t temp_selected_polynomial = db_lh2_polynomial(temp_bits_sweep, &temp_bit_offset);
    if (temp_selected_polynomial == DB_LH2_POLYNOMIAL_NONE) {
        return DB_LH2_DECODE_NO_POLYNOMIAL;
    }

    uint8_t        sweep       = _select_sweep(dec, temp_selected_polynomial, cap->t_us, now_us);
    uint8_t        basestation = temp_selected_polynomial >> 1;
    db_lh2_slot_t *slot        = &dec->slot[sweep][basestation];

    // The 17-bit seed the polynomial match settled on
    uint32_t temp_lfsr_bits = temp_bits_sweep >> (47 - temp_bit_offset);
    if (temp_lfsr_bits == 0x000000) {
        slot->ready = false;
        return DB_LH2_DECODE_ZERO_BITS;
    }

    uint32_t temp_lfsr_loc = db_lh2_lfsr_index(dec, temp_selected_polynomial, temp_lfsr_bits);
    if (temp_lfsr_loc == DB_LH2_INDEX_NONE) {
        slot->ready = false;
        return DB_LH2_DECODE_NO_INDEX;
    }
    dec->checkpoint_bits[temp_selected_polynomial][sweep]  = temp_lfsr_bits;
    dec->checkpoint_count[temp_selected_polynomial][sweep] = temp_lfsr_loc;

    // The count of the first bit that hit the sensor
    temp_lfsr_loc -= temp_bit_offset;

    // A count past a full rotation, or equal to the other sweep's, comes from a false polynomial match
    if (temp_lfsr_loc > db_lh2_period(basestation) / DB_LH2_PERIOD_TICKS_PER_COUNT) {
        slot->ready = false;
        return DB_LH2_DECODE_PAST_REVOLUTION;
    }
    db_lh2_slot_t *other = &dec->slot[sweep ^ 1][basestation];
    if (other->ready && other->count == temp_lfsr_loc) {
        slot->ready  = false;
        other->ready = false;
        return DB_LH2_DECODE_EQUAL_COUNTS;
    }

    slot->t_us       = cap->t_us;
    slot->count      = temp_lfsr_loc;
    slot->polynomial = temp_selected_polynomial;
    slot->ready      = true;
    return DB_LH2_DECODE_STORED;
}

//=========================== decoder ==========================================

void db_lh2_decoder_init(db_lh2_decoder_t *dec) {
    memset(dec, 0, sizeof(*dec));
    for (uint8_t sweep = 0; sweep < DB_LH2_SWEEPS; sweep++) {
        for (uint8_t station = 0; station < DB_LH2_STATIONS; station++) {
            dec->slot[sweep][station].count      = DB_LH2_INDEX_NONE;
            dec->slot[sweep][station].polynomial = DB_LH2_POLYNOMIAL_NONE;
        }
    }
}

void db_lh2_decoder_reset(db_lh2_decoder_t *dec) {
    for (uint8_t station = 0; station < DB_LH2_STATIONS; station++) {
        for (uint8_t sweep = 0; sweep < DB_LH2_SWEEPS; sweep++) {
            dec->slot[sweep][station].ready = false;
            dec->slot[sweep][station].t_us  = 0;
        }
    }
}

db_lh2_decode_result_t db_lh2_decoder_feed(db_lh2_decoder_t *dec, const db_lh2_capture_t *cap, uint32_t now_us) {
    db_lh2_decode_result_t result = _decode(dec, cap, now_us);
    dec->results[result]++;
    return result;
}

bool db_lh2_decoder_pair_ready(const db_lh2_decoder_t *dec, uint8_t station) {
    return station < DB_LH2_STATIONS && dec->slot[0][station].ready && dec->slot[1][station].ready;
}

bool db_lh2_decoder_take_pair(db_lh2_decoder_t *dec, uint8_t station, db_lh2_pair_t *out) {
    if (!db_lh2_decoder_pair_ready(dec, station)) {
        return false;
    }
    out->station = station;
    for (uint8_t sweep = 0; sweep < DB_LH2_SWEEPS; sweep++) {
        out->count[sweep]               = dec->slot[sweep][station].count;
        out->t_us[sweep]                = dec->slot[sweep][station].t_us;
        dec->slot[sweep][station].ready = false;
    }
    return true;
}

//=========================== queue ============================================

void db_lh2_queue_clear(db_lh2_queue_t *queue) {
    queue->write = 0;
    queue->read  = 0;
    queue->count = 0;
}

void db_lh2_queue_push(db_lh2_queue_t *queue, const uint8_t spi[DB_LH2_CAPTURE_BYTES], uint32_t t_us) {
    memcpy(queue->capture[queue->write].spi, spi, DB_LH2_CAPTURE_BYTES);
    queue->capture[queue->write].t_us = t_us;
    queue->write                      = (queue->write + 1) % DB_LH2_QUEUE_LEN;
    if (queue->count < DB_LH2_QUEUE_LEN) {
        queue->count++;
    } else {
        queue->read = (queue->read + 1) % DB_LH2_QUEUE_LEN;
        queue->dropped++;
    }
}

bool db_lh2_queue_pop(db_lh2_queue_t *queue, db_lh2_capture_t *out) {
    if (queue->count == 0) {
        return false;
    }
    *out        = queue->capture[queue->read];
    queue->read = (queue->read + 1) % DB_LH2_QUEUE_LEN;
    queue->count--;
    return true;
}
