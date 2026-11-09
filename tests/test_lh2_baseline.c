/**
 * @file
 * @brief  Host test pinning today's LH2 decode path, defects included
 *
 * Every check runs the frozen 228ce9a copy in lh2_reference.c on captures
 * from lh2_synth.c. A check that pins a defect says so in its message; when a
 * later change fixes one, its check is updated in the same change, as a
 * declared difference from today. Measured numbers are printed so a run can be
 * compared with the last within noise. `--long` runs the full-size sweeps.
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lh2_geometry.h"
#include "lh2_reference.h"
#include "lh2_synth.h"

//=========================== harness ==========================================

static int  _failed = 0;
static int  _passed = 0;
static bool _long   = false;

#define CHECK(cond, ...)                                \
    do {                                                \
        if (cond) {                                     \
            _passed++;                                  \
        } else {                                        \
            _failed++;                                  \
            printf("FAIL %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                        \
            printf("\n");                               \
        }                                               \
    } while (0)

#define MEASURED(...)               \
    do {                            \
        printf("  lh2_baseline: "); \
        printf(__VA_ARGS__);        \
        printf("\n");               \
    } while (0)

#define T_US (5000000U)  ///< a capture's stamp, where the time does not matter

static lh2_ref_t         _lh2;
static lh2_synth_event_t _events[40000];

static void _hex(const char *what, const uint8_t spi[LH2_SYNTH_CAPTURE_BYTES]) {
    printf("  %s:", what);
    for (uint32_t i = 0; i < LH2_SYNTH_CAPTURE_BYTES; i++) {
        printf("%02x", spi[i]);
    }
    printf("\n");
}

//=========================== helpers ==========================================

typedef struct {
    lh2_ref_result_t result;
    uint8_t          polynomial;
    uint32_t         count;
} decoded_t;

/// One capture into a cold decoder: both slots empty, so a stored one takes slot 0
static decoded_t _decode_cold(const uint8_t spi[LH2_SYNTH_CAPTURE_BYTES]) {
    lh2_ref_init(&_lh2);
    decoded_t out = { .result = lh2_ref_feed(&_lh2, spi, T_US, T_US + 50), .polynomial = LH2_REF_POLYNOMIAL_ERROR, .count = 0 };
    if (out.result == LH2_REF_STORED) {
        for (uint8_t station = 0; station < LH2_REF_STATIONS; station++) {
            if (_lh2.data_ready[0][station] == LH2_REF_PROCESSED_DATA_AVAILABLE) {
                out.polynomial = (uint8_t)_lh2.locations[0][station].selected_polynomial;
                out.count      = _lh2.locations[0][station].lfsr_counts;
            }
        }
    }
    return out;
}

/// LFSR counts per revolution of a station
static uint32_t _revolution(uint8_t station) {
    return db_lh2_period(station) / DB_LH2_PERIOD_TICKS_PER_COUNT;
}

/// A clean pass anywhere in its station's revolution
static lh2_synth_light_t _random_light(lh2_synth_rng_t *rng) {
    lh2_synth_light_t light = { 0 };
    light.polynomial        = (uint8_t)(lh2_synth_rand(rng) >> 27);
    light.count             = 20U + (uint32_t)(lh2_synth_uniform(rng) * (_revolution(light.polynomial >> 1) - 200U));
    light.phase             = lh2_synth_uniform(rng) * LH2_SYNTH_SAMPLES_PER_BIT;
    light.level             = (uint8_t)(lh2_synth_rand(rng) >> 31);
    return light;
}

/// Samples before the first transition
static uint32_t _first_run(const uint8_t spi[LH2_SYNTH_CAPTURE_BYTES]) {
    uint32_t run   = 0;
    uint8_t  first = spi[0] >> 7;
    while (run < LH2_SYNTH_SAMPLES && ((spi[run / 8] >> (7 - run % 8)) & 1U) == first) {
        run++;
    }
    return run;
}

/// The count the decode chain gives a capture, through the low layer and a cold search
static uint32_t _chain_count(const uint8_t spi[LH2_SYNTH_CAPTURE_BYTES], uint8_t *polynomial) {
    lh2_ref_checkpoint_t cold   = { 0 };
    uint64_t             bits   = lh2_ref_demodulate(spi);
    int8_t               offset = 0;
    *polynomial                 = lh2_ref_determine_polynomial(bits, &offset);
    if (*polynomial == LH2_REF_POLYNOMIAL_ERROR) {
        return LH2_REF_INDEX_ERROR;
    }
    return lh2_ref_lfsr_index_search(&cold, *polynomial, (uint32_t)(bits >> (47 - offset))) - (uint32_t)offset;
}

/// The 64-bit word a perfect demodulator gives for stream bits n .. n + 63
static uint64_t _word(uint8_t polynomial, uint32_t n) {
    uint8_t bits[64];
    lh2_synth_lfsr_bits(polynomial, n, bits, 64);
    uint64_t word = 0;
    for (uint32_t i = 0; i < 64; i++) {
        word = (word << 1) | bits[i];
    }
    return word;
}

/// What the swarmit glue does to read a station: both sweeps fresh, then cleared
static bool _take_pair(lh2_ref_t *lh2, uint8_t station, uint32_t counts[2]) {
    if (lh2->data_ready[0][station] != LH2_REF_PROCESSED_DATA_AVAILABLE || lh2->data_ready[1][station] != LH2_REF_PROCESSED_DATA_AVAILABLE) {
        return false;
    }
    counts[0]                   = lh2->locations[0][station].lfsr_counts;
    counts[1]                   = lh2->locations[1][station].lfsr_counts;
    lh2->data_ready[0][station] = LH2_REF_NO_NEW_DATA;
    lh2->data_ready[1][station] = LH2_REF_NO_NEW_DATA;
    return true;
}

/// The slot a stored event landed in, or -1
static int _slot_of(const lh2_ref_t *lh2, const lh2_synth_event_t *ev) {
    for (int sweep = 0; sweep < 2; sweep++) {
        if (lh2->data_ready[sweep][ev->station] == LH2_REF_PROCESSED_DATA_AVAILABLE && lh2->timestamps[sweep][ev->station] == ev->t_us) {
            return sweep;
        }
    }
    return -1;
}

static bool _near(uint32_t count, uint32_t n0) {
    return count == n0 || count == n0 + 1U;
}

static bool _within2(uint32_t count, uint32_t n0) {
    return count + 2U >= n0 && count <= n0 + 2U;
}

//=========================== the generator ====================================

static void test_synth_polynomials_and_lfsr(void) {
    bool same = true;
    for (uint8_t p = 0; p < LH2_SYNTH_POLYNOMIALS; p++) {
        same = same && lh2_synth_polynomial(p) == lh2_ref_polynomial(p);
    }
    CHECK(same, "the generator's polynomials are the decoder's");

    // The 2,048 static checkpoints are the LFSR state at their index, in the slot of its top 6 bits
    uint32_t wrong = 0;
    for (uint8_t p = 0; p < LH2_SYNTH_POLYNOMIALS; p++) {
        for (uint8_t slot = 0; slot < LH2_REF_CHECKPOINTS; slot++) {
            uint32_t state = lh2_synth_lfsr_state(p, lh2_ref_static_checkpoint(p, slot));
            wrong += state != lh2_ref_static_checkpoint_state(p, slot) || ((state >> 11) & 0x3FU) != slot;
        }
    }
    CHECK(wrong == 0, "every static checkpoint is the LFSR state at its index: %u of 2048 are not", wrong);

    uint8_t bits[40];
    lh2_synth_lfsr_bits(5, 1000, bits, 40);
    uint32_t window = 0;
    for (uint32_t i = 3; i < 20; i++) {
        window = (window << 1) | bits[i];
    }
    CHECK(window == lh2_synth_lfsr_state(5, 1003) && lh2_synth_lfsr_index(5, window) == 1003, "stream bits n .. n + 16 are the state at n, MSB oldest");
    CHECK(lh2_synth_lfsr_state(0, 0) == 1 && lh2_synth_lfsr_state(0, LH2_SYNTH_LFSR_PERIOD) == 1, "the LFSR starts at state 1 and repeats after 2^17 - 1");
}

static void test_synth_code(void) {
    // Biphase mark: a run of 32/6 samples for a 0, two of 16/6 for a 1
    uint8_t bits[LH2_SYNTH_BITS] = { 0 };
    uint8_t spi[LH2_SYNTH_CAPTURE_BYTES];
    lh2_synth_code(bits, 0.0, 0, 0.0, NULL, spi);
    uint32_t edges = 0;
    for (uint32_t k = 1; k < LH2_SYNTH_SAMPLES; k++) {
        edges += ((spi[k / 8] >> (7 - k % 8)) & 1U) != ((spi[(k - 1) / 8] >> (7 - (k - 1) % 8)) & 1U);
    }
    CHECK(edges == 95, "all zeros: one edge per bit boundary inside 512 samples, got %u", edges);
    memset(bits, 1, sizeof(bits));
    lh2_synth_code(bits, 0.0, 0, 0.0, NULL, spi);
    edges = 0;
    for (uint32_t k = 1; k < LH2_SYNTH_SAMPLES; k++) {
        edges += ((spi[k / 8] >> (7 - k % 8)) & 1U) != ((spi[(k - 1) / 8] >> (7 - (k - 1) % 8)) & 1U);
    }
    CHECK(edges == 191, "all ones: two edges per bit inside 512 samples, got %u", edges);
}

static void test_synth_knobs(void) {
    lh2_synth_rng_t   rng   = { 1001 };
    lh2_synth_light_t light = _random_light(&rng);
    uint8_t           clean[LH2_SYNTH_CAPTURE_BYTES], spi[LH2_SYNTH_CAPTURE_BYTES];
    lh2_synth_capture(&light, &rng, clean);

    // flips: exactly k samples differ
    memcpy(spi, clean, sizeof(spi));
    lh2_synth_flip_samples(spi, 7, &rng);
    uint32_t differ = 0;
    for (uint32_t i = 0; i < LH2_SYNTH_CAPTURE_BYTES; i++) {
        differ += (uint32_t)__builtin_popcount(spi[i] ^ clean[i]);
    }
    CHECK(differ == 7, "flips: 7 samples inverted, %u differ", differ);

    // jitter: none at sigma 0, and the same number of edges at a small sigma
    lh2_synth_light_t jittered = light;
    lh2_synth_capture(&jittered, &rng, spi);
    CHECK(memcmp(spi, clean, sizeof(spi)) == 0, "jitter 0 changes nothing");
    jittered.jitter = 0.2;
    lh2_synth_capture(&jittered, &rng, spi);
    CHECK(memcmp(spi, clean, sizeof(spi)) != 0, "jitter moves edges");

    // dwell: the light up to it unchanged, the idle level from it on
    lh2_synth_light_t dwell = light;
    dwell.dwell_us          = 9.0;
    dwell.idle_level        = 1;
    lh2_synth_capture(&dwell, &rng, spi);
    CHECK(memcmp(spi, clean, 36) == 0, "dwell: the first 9 us untouched");
    bool idle = true;
    for (uint32_t i = 36; i < LH2_SYNTH_CAPTURE_BYTES; i++) {
        idle = idle && spi[i] == 0xFF;
    }
    CHECK(idle, "dwell: the idle level from 9 us on");

    // mocap: a square wave of the period asked for
    lh2_synth_mocap(spi, 32, 0);
    CHECK(spi[0] == 0xFF && spi[1] == 0xFF && spi[2] == 0x00 && spi[3] == 0x00 && spi[4] == 0xFF, "mocap: 1 MHz is two bytes high, two low");

    // collide: the first capture's samples up to the offset, the second's after
    lh2_synth_light_t other = _random_light(&rng);
    uint8_t           second[LH2_SYNTH_CAPTURE_BYTES];
    lh2_synth_capture(&other, &rng, second);
    lh2_synth_collide(clean, second, 4.0, spi);
    CHECK(memcmp(spi, clean, 16) == 0 && memcmp(spi + 16, second + 16, 48) == 0, "collide at 4 us switches at byte 16");

    // confuse: the flips make the window the other polynomial's run from the same seed
    uint8_t where[39];
    uint8_t other_poly = (uint8_t)((light.polynomial + 7) % 32);
    uint8_t n          = lh2_synth_confuse(&light, 0, other_poly, where);
    CHECK(n > 0 && n <= 39, "confuse: some flips aimed, %u", n);
    lh2_synth_light_t aimed = light;
    for (uint8_t i = 0; i < n; i++) {
        aimed.flip[where[i] / 32] |= 1U << (where[i] % 32);
    }
    uint8_t bits[LH2_SYNTH_BITS];
    lh2_synth_lfsr_bits(aimed.polynomial, aimed.count, bits, LH2_SYNTH_BITS);
    uint32_t seed = 0;
    for (uint32_t j = 8; j < 25; j++) {
        seed = (seed << 1) | bits[j];
    }
    bool matches = true;
    for (uint32_t j = 25; j < 64; j++) {
        uint32_t next = __builtin_popcount(seed & lh2_synth_polynomial(other_poly)) & 1U;
        seed          = ((seed << 1) | next) & 0x1FFFFU;
        uint8_t got   = bits[j] ^ (uint8_t)((aimed.flip[j / 32] >> (j % 32)) & 1U);
        matches       = matches && got == next;
    }
    CHECK(matches, "confuse: the flipped window is the other polynomial's sequence");
}

static void test_synth_stream(void) {
    lh2_synth_rng_t     rng = { 1002 };
    lh2_synth_station_t st  = { .station = 1, .counts = { 30000, 70000 }, .t0_us = 1000, .dup_share = 1.0, .dup_delay_us = { 33, 33 }, .dup_sweeps = 1, .latency_us = 0 };
    size_t              n   = lh2_synth_stream(&st, 1, 10 * lh2_synth_period_us(1), 0x100000000ULL - 100000ULL, &rng, _events, 40000);
    CHECK(n == 30, "dup-late on sweep 0 of every pass: 3 captures a revolution, 10 revolutions, got %zu", n);
    bool ordered = true, dups = true, wrapped = false;
    for (size_t i = 1; i < n; i++) {
        ordered = ordered && _events[i].t_true_us >= _events[i - 1].t_true_us;
        wrapped = wrapped || _events[i].t_us < _events[i - 1].t_us;
        if (_events[i].duplicate) {
            dups = dups && _events[i].count == _events[i - 1].count + 198U && _events[i].t_us - _events[i - 1].t_us >= 32U && _events[i].t_us - _events[i - 1].t_us <= 34U;
        }
    }
    CHECK(ordered, "stream: arrival order");
    CHECK(dups, "dup-late: 33 us later, 198 counts on");
    CHECK(wrapped, "wrap: a base near 2^32 wraps the stamps inside the run");

    lh2_synth_station_t late = { .station = 1, .counts = { 30000, 70000 }, .t0_us = 1000, .latency_us = 500, .drop_share = 0.5 };
    n                        = lh2_synth_stream(&late, 1, 200 * lh2_synth_period_us(1), 0, &rng, _events, 40000);
    bool bounded             = true;
    for (size_t i = 0; i < n; i++) {
        double lag = _events[i].t_us - (_events[i].t_true_us + LH2_SYNTH_CAPTURE_US);
        bounded    = bounded && lag > -1.0 && lag <= 500.0;
    }
    CHECK(bounded, "latency: stamps between the capture's end and 500 us after");
    CHECK(n > 120 && n < 280, "drop: about half of 400 passes kept, %zu", n);
}

//=========================== drv/lh2_decoder: from bits to a count ============

static void test_d1_clean(void) {
    lh2_synth_rng_t rng      = { 2001 };
    uint32_t        per_poly = 625;
    uint32_t        total = 0, decoded = 0, wrong = 0, far = 0, at_n0 = 0;
    uint32_t        positions = 0;
    for (uint8_t p = 0; p < LH2_REF_POLYNOMIALS; p++) {
        uint32_t rev = _revolution(p >> 1);
        for (uint32_t i = 0; i < per_poly; i++) {
            uint32_t n0;
            if (i == 0) {
                n0 = 0;
            } else if (i == 1) {
                n0 = 17;
            } else if (i == 2) {
                n0 = rev - 2;  // so n0 + 1 is the revolution's last count
            } else if (i < 3 + 2 * LH2_REF_CHECKPOINTS) {
                uint32_t c = lh2_ref_static_checkpoint(p, (uint8_t)((i - 3) / 2));
                n0         = ((i - 3) % 2) ? c + 1U : (c ? c - 1U : 0);
            } else if (i < 400) {
                n0 = 108326U + (uint32_t)(lh2_synth_uniform(&rng) * (rev - 1U - 108326U));
            } else {
                n0 = (uint32_t)(lh2_synth_uniform(&rng) * (rev - 1U));
            }
            positions++;
            lh2_synth_light_t light = { .polynomial = p, .count = n0, .phase = lh2_synth_uniform(&rng) * LH2_SYNTH_SAMPLES_PER_BIT, .level = (uint8_t)(lh2_synth_rand(&rng) >> 31) };
            uint8_t           spi[LH2_SYNTH_CAPTURE_BYTES];
            lh2_synth_capture(&light, &rng, spi);
            decoded_t d = _decode_cold(spi);
            total++;
            if (d.result != LH2_REF_STORED) {
                continue;
            }
            decoded++;
            if (d.polynomial != p) {
                wrong++;
                _hex("D1 wrong station", spi);
            } else if (!_near(d.count, n0)) {
                far++;
            } else {
                at_n0 += d.count == n0;
            }
        }
    }
    MEASURED("D1 clean: %u of %u decoded, %u wrong station, %u off n0 / n0 + 1, %u at n0", decoded, total, wrong, far, at_n0);
    CHECK(total == 20000 && positions == 20000, "D1: 20,000 clean captures");
    CHECK(wrong == 0, "D1: no clean capture decodes to another station, %u did", wrong);
    CHECK(far == 0, "D1: every decoded count is n0 or n0 + 1, %u were not", far);
    // Defect pinned: a few clean captures do not decode at all
    CHECK(total - decoded >= 1 && total - decoded <= 40, "D1 (defect): a handful of clean captures not decoded, %u of 20,000", total - decoded);
}

static void test_d2_start_phase(void) {
    lh2_synth_rng_t rng       = { 2002 };
    uint32_t        short_run = 0, short_n0 = 0, long_run = 0, long_n1 = 0, four = 0, four_n0 = 0, other = 0;
    for (uint32_t i = 0; i < 200; i++) {
        lh2_synth_light_t light = _random_light(&rng);
        for (uint32_t step = 0; step < 86; step++) {
            light.phase = step / 16.0;
            uint8_t spi[LH2_SYNTH_CAPTURE_BYTES];
            lh2_synth_capture(&light, &rng, spi);
            uint32_t  run = _first_run(spi);
            decoded_t d   = _decode_cold(spi);
            if (d.result != LH2_REF_STORED || d.polynomial != light.polynomial || !_near(d.count, light.count)) {
                other++;
                continue;
            }
            bool at_n0 = d.count == light.count;
            if (run <= 3) {
                short_run++;
                short_n0 += at_n0;
            } else if (run == 4) {
                four++;
                four_n0 += at_n0;
            } else {
                long_run++;
                long_n1 += !at_n0;
            }
        }
    }
    MEASURED("D2 phase: first run 1-3 samples %u (%u at n0), 4 samples %u (%u at n0), 5+ samples %u (%u at n0 + 1), %u not n0 / n0 + 1", short_run, short_n0, four, four_n0, long_run, long_n1, other);
    CHECK(short_n0 == 0 && short_run > 0, "D2: a first run of 3 samples or fewer is dropped, the count is n0 + 1");
    CHECK(long_n1 == 0 && long_run > 0, "D2: a first run of 5 samples or more is kept, the count is n0");
    CHECK(four_n0 > 0 && four_n0 < four, "D2: a first run of exactly 4 samples goes either way");
}

static void test_d3_sample_flips(void) {
    lh2_synth_rng_t rng   = { 2003 };
    uint32_t        per_k = _long ? 20000 : 4000;
    uint32_t        wrong = 0, accepted = 0, beyond = 0, far = 0;
    uint32_t        acc4 = 0, beyond4 = 0, far4 = 0;
    for (uint32_t k = 1; k <= 12; k++) {
        for (uint32_t i = 0; i < per_k; i++) {
            lh2_synth_light_t light = _random_light(&rng);
            uint8_t           spi[LH2_SYNTH_CAPTURE_BYTES];
            lh2_synth_capture(&light, &rng, spi);
            lh2_synth_flip_samples(spi, k, &rng);
            decoded_t d = _decode_cold(spi);
            if (d.result != LH2_REF_STORED) {
                continue;
            }
            if (d.polynomial != light.polynomial) {
                wrong++;
                continue;
            }
            accepted++;
            int32_t e = (int32_t)d.count - (int32_t)light.count;
            if (e < -2 || e > 2) {
                far++;
                far4 += k <= 4;
            } else if (e != 0 && e != 1) {
                beyond++;
                beyond4 += k <= 4;
            }
            acc4 += k <= 4;
        }
    }
    MEASURED("D3 flips 1-12, %u each: %u accepted, %u wrong station, %u off n0 / n0 + 1 within 2, %u further; at 4 flips or fewer %u of %u off", per_k, accepted, wrong, beyond, far, beyond4, acc4);
    CHECK(far4 == 0, "D3: under 4 flips or fewer an accepted count stays within 2 of n0, %u did not", far4);
    CHECK(far <= per_k * 12 / 40000 + 2, "D3: past 4 flips a count more than 2 off stays rare, %u", far);
    CHECK(beyond > 0, "D3: flips move some accepted counts by 1 or 2 past n0 / n0 + 1");
    // Defect pinned: random flips rarely, but not never, make another station
    CHECK(wrong <= per_k * 12 / 4000, "D3 (defect): wrong stations stay rare, %u in %u", wrong, per_k * 12);
}

static void test_d3_dead_tolerance(void) {
    // A match needs every compared bit: the threshold is 0, whatever the BSP's dead define of 4 says
    uint8_t  p = 9;
    uint32_t n = 50000;
    int8_t   offset;
    uint64_t clean = _word(p, n);
    CHECK(lh2_ref_determine_polynomial(clean, &offset) == p && offset == 8, "a clean word matches at offset 8");
    uint32_t rejected = 0, kept_late = 0;
    for (uint32_t bit = 25; bit < 64; bit++) {
        uint8_t got = lh2_ref_determine_polynomial(clean ^ (1ULL << (63 - bit)), &offset);
        if (bit < 44) {
            rejected += got == LH2_REF_POLYNOMIAL_ERROR;
        } else {
            kept_late += got == p;
        }
    }
    CHECK(rejected == 19, "dead tolerance: one wrong bit in word bits 25-43, which every window compares, rejects the capture: %u of 19", rejected);
    CHECK(kept_late == 20, "one wrong bit in word bits 44-63 is dropped with the window end: %u of 20 still match", kept_late);
}

static void test_d4_confuse(void) {
    lh2_synth_rng_t rng   = { 2004 };
    uint32_t        cases = 0, other = 0, own = 0, none = 0, min_flips = 99;
    for (uint32_t i = 0; i < 300; i++) {
        lh2_synth_light_t light = _random_light(&rng);
        uint8_t           spi[LH2_SYNTH_CAPTURE_BYTES];
        lh2_synth_capture(&light, &rng, spi);
        decoded_t d = _decode_cold(spi);
        if (d.result != LH2_REF_STORED || d.polynomial != light.polynomial) {
            continue;
        }
        uint8_t shift = (uint8_t)(d.count - light.count);
        uint8_t target;
        do {
            target = (uint8_t)(lh2_synth_rand(&rng) >> 27);
        } while ((target >> 1) == (light.polynomial >> 1));
        uint8_t where[39];
        uint8_t n = lh2_synth_confuse(&light, shift, target, where);
        for (uint8_t k = 1; k <= n; k++) {
            lh2_synth_light_t aimed = light;
            for (uint8_t j = 0; j < k; j++) {
                aimed.flip[where[j] / 32] |= 1U << (where[j] % 32);
            }
            lh2_synth_capture(&aimed, &rng, spi);
            decoded_t a = _decode_cold(spi);
            cases++;
            if (a.result != LH2_REF_STORED) {
                none++;
            } else if (a.polynomial == target) {
                other++;
                min_flips = (k < min_flips) ? k : min_flips;
            } else if (a.polynomial == light.polynomial) {
                own++;
            }
        }
    }
    MEASURED("D4 confuse: %u aimed captures, %u read as the aimed-at station (from %u bit flips), %u as their own, %u rejected", cases, other, min_flips, own, none);
    // Defect pinned: flipping the bits the shortest window compares makes the other station
    CHECK(other > 0, "D4 (defect): aimed flips can make another station");
}

static void test_d5_jitter(void) {
    lh2_synth_rng_t rng      = { 2005 };
    const double    sigmas[] = { 0.3, 0.6, 1.0 };
    uint32_t        n        = 4000;
    uint32_t        rate[3], wrong = 0;
    for (uint32_t s = 0; s < 3; s++) {
        rate[s] = 0;
        for (uint32_t i = 0; i < n; i++) {
            lh2_synth_light_t light = _random_light(&rng);
            light.jitter            = sigmas[s];
            uint8_t spi[LH2_SYNTH_CAPTURE_BYTES];
            lh2_synth_capture(&light, &rng, spi);
            decoded_t d = _decode_cold(spi);
            if (d.result == LH2_REF_STORED) {
                rate[s] += d.polynomial == light.polynomial;
                wrong += d.polynomial != light.polynomial;
            }
        }
    }
    MEASURED("D5 jitter sigma 0.3 / 0.6 / 1.0 samples: %u / %u / %u of %u decoded, %u wrong station", rate[0], rate[1], rate[2], n, wrong);
    CHECK(rate[0] > n * 80 / 100 && rate[1] < n * 40 / 100 && rate[2] < n * 2 / 100, "D5: the decode rate falls fast with edge jitter");
    CHECK(wrong <= 2, "D5: jitter makes another station rarely, %u", wrong);
}

static void test_d6_mocap_filter_on_by_default(void) {
    // LH2_MOCAP_FILTER is defined at 228ce9a: a capture whose light ends before
    // 7.75 us leaves three idle bytes in the first half and is taken for a
    // motion-capture strobe. Today's default; this pin changes when it does.
    lh2_synth_rng_t rng      = { 2006 };
    const double    dwells[] = { 2, 4, 6, 7, 7.75, 8, 9, 12, 16 };
    uint32_t        n        = 2000;
    for (uint32_t s = 0; s < sizeof(dwells) / sizeof(dwells[0]); s++) {
        uint32_t mocap = 0, ok = 0, wrong = 0;
        for (uint32_t i = 0; i < n; i++) {
            lh2_synth_light_t light = _random_light(&rng);
            light.dwell_us          = dwells[s];
            light.idle_level        = (uint8_t)(lh2_synth_rand(&rng) >> 31);
            uint8_t spi[LH2_SYNTH_CAPTURE_BYTES];
            lh2_synth_capture(&light, &rng, spi);
            decoded_t d = _decode_cold(spi);
            mocap += d.result == LH2_REF_MOCAP;
            ok += d.result == LH2_REF_STORED && d.polynomial == light.polynomial && _near(d.count, light.count);
            wrong += d.result == LH2_REF_STORED && d.polynomial != light.polynomial;
        }
        MEASURED("D6 dwell %5.2f us: %u of %u rejected as mocap, %u decoded right, %u wrong station", dwells[s], mocap, n, ok, wrong);
        if (dwells[s] <= 7.75) {
            CHECK(mocap == n, "D6 mocap filter ON (today's default): every dwell of %.2f us rejected as mocap, %u of %u", dwells[s], mocap, n);
        } else if (dwells[s] == 8) {
            CHECK(mocap == 0 && ok > n * 90 / 100 && ok < n * 99 / 100, "D6: about 95%% decoded at 8 us, %u of %u", ok, n);
        } else {
            CHECK(mocap == 0 && ok >= n * 995 / 1000, "D6: 99.5%% or more decoded at %.0f us, %u of %u", dwells[s], ok, n);
        }
        CHECK(wrong == 0, "D6: a short dwell never makes another station");
    }
}

static void test_d7_mocap_strobe(void) {
    uint32_t results[3][7] = { { 0 } };
    uint32_t periods[3]    = { 16, 32, 48 };  // 2 MHz, 1 MHz, and the 3-byte pattern the filter's comment names
    bool     station0_hit  = false;
    for (uint32_t i = 0; i < 3; i++) {
        for (uint32_t phase = 0; phase < periods[i]; phase++) {
            uint8_t spi[LH2_SYNTH_CAPTURE_BYTES];
            lh2_synth_mocap(spi, periods[i], phase);
            lh2_ref_init(&_lh2);
            _lh2.data_ready[0][0] = LH2_REF_PROCESSED_DATA_AVAILABLE;
            _lh2.timestamps[0][0] = T_US - 100000;
            lh2_ref_result_t r    = lh2_ref_feed(&_lh2, spi, T_US, T_US + 50);
            results[i][r]++;
            station0_hit = station0_hit || (i == 0 && _lh2.data_ready[0][0] != LH2_REF_PROCESSED_DATA_AVAILABLE);
        }
    }
    MEASURED("D7 strobe, 2 MHz: %u mocap, %u no polynomial, %u zero bits; 1 MHz: %u mocap, %u no polynomial; 48 samples: %u mocap, %u no polynomial",
             results[0][LH2_REF_MOCAP], results[0][LH2_REF_NO_POLYNOMIAL], results[0][LH2_REF_ZERO_BITS], results[1][LH2_REF_MOCAP], results[1][LH2_REF_NO_POLYNOMIAL], results[2][LH2_REF_MOCAP], results[2][LH2_REF_NO_POLYNOMIAL]);
    CHECK(results[0][LH2_REF_STORED] + results[1][LH2_REF_STORED] + results[2][LH2_REF_STORED] == 0, "D7: no strobe is stored");
    CHECK(results[1][LH2_REF_NO_POLYNOMIAL] == 32, "D7: a 1 MHz strobe passes the mocap check and fails at the polynomial, slots untouched");
    CHECK(results[2][LH2_REF_MOCAP] > 0, "D7: the 3-byte pattern the mocap check was written for is caught");
    // Defect pinned: a 2 MHz strobe can match polynomial 0 on an all-zero seed and clear station 0's slot
    CHECK(results[0][LH2_REF_ZERO_BITS] > 0 && station0_hit, "D7 (defect): a 2 MHz strobe reaches the zero-bits check and clears station 0's slot");
}

static void test_d8_collide(void) {
    lh2_synth_rng_t rng       = { 2008 };
    uint32_t        n         = 1000;
    uint32_t        third     = 0;
    uint32_t        first[17] = { 0 }, second[17] = { 0 };
    for (uint32_t offset = 0; offset <= 16; offset += 2) {
        for (uint32_t i = 0; i < n; i++) {
            lh2_synth_light_t a = _random_light(&rng), b;
            do {
                b = _random_light(&rng);
            } while ((b.polynomial >> 1) == (a.polynomial >> 1));
            uint8_t sa[LH2_SYNTH_CAPTURE_BYTES], sb[LH2_SYNTH_CAPTURE_BYTES], spi[LH2_SYNTH_CAPTURE_BYTES];
            lh2_synth_capture(&a, &rng, sa);
            lh2_synth_capture(&b, &rng, sb);
            lh2_synth_collide(sa, sb, offset, spi);
            decoded_t d = _decode_cold(spi);
            if (d.result != LH2_REF_STORED) {
                continue;
            }
            if (d.polynomial == a.polynomial && _within2(d.count, a.count)) {
                first[offset]++;
            } else if (d.polynomial == b.polynomial && _within2(d.count, b.count)) {
                second[offset]++;
            } else {
                third++;
            }
        }
    }
    MEASURED("D8 collide, first / second decoded right of %u, offset 0-16 us: 0 %u/%u, 2 %u/%u, 4 %u/%u, 6 %u/%u, 8 %u/%u, 10 %u/%u, 16 %u/%u; %u neither", n,
             first[0], second[0], first[2], second[2], first[4], second[4], first[6], second[6], first[8], second[8], first[10], second[10], first[16], second[16], third);
    CHECK(third == 0, "D8: a collision decodes to one of its two stations, within 2 of its count, or not at all; %u did not", third);
    CHECK(first[4] + second[4] + first[6] + second[6] < n / 50, "D8: a takeover 4 to 6 us in leaves neither station decodable");
}

static void test_d9_constant_captures(void) {
    uint8_t spi[LH2_SYNTH_CAPTURE_BYTES];
    memset(spi, 0x00, sizeof(spi));
    CHECK(_decode_cold(spi).result == LH2_REF_MOCAP, "D9: an all-zero capture is taken for mocap");
    memset(spi, 0xFF, sizeof(spi));
    CHECK(_decode_cold(spi).result == LH2_REF_MOCAP, "D9: an all-one capture is taken for mocap");

    // All-zero bits still code to a square wave, which the demodulator reads as zero chips
    uint8_t bits[LH2_SYNTH_BITS] = { 0 };
    lh2_synth_code(bits, 0.0, 0, 0.0, NULL, spi);
    CHECK(_decode_cold(spi).result == LH2_REF_NO_POLYNOMIAL, "D9: an all-zero bit stream has no polynomial");

    // Zeros through the shortest window, ones after: every polynomial runs an
    // all-zero seed to zeros, polynomial 0 wins, and the zero-bits check fires
    for (uint32_t i = 44; i < LH2_SYNTH_BITS; i++) {
        bits[i] = 1;
    }
    lh2_synth_code(bits, 0.0, 0, 0.0, NULL, spi);
    lh2_ref_init(&_lh2);
    _lh2.data_ready[0][0] = LH2_REF_PROCESSED_DATA_AVAILABLE;
    _lh2.timestamps[0][0] = T_US - 100000;
    lh2_ref_result_t r    = lh2_ref_feed(&_lh2, spi, T_US, T_US + 50);
    CHECK(r == LH2_REF_ZERO_BITS, "D9: an all-zero seed is rejected by the zero-bits check, got %d", r);
    CHECK(_lh2.data_ready[0][0] == LH2_REF_NO_NEW_DATA, "D9 (defect): and clears a slot of station 0, which sent nothing");
}

static void test_d10_random_captures(void) {
    lh2_synth_rng_t rng        = { 2010 };
    uint32_t        n          = _long ? 1000000 : 100000;
    uint32_t        results[8] = { 0 };
    bool            sane       = true;
    lh2_ref_init(&_lh2);
    for (uint32_t i = 0; i < n; i++) {
        uint8_t spi[LH2_SYNTH_CAPTURE_BYTES];
        for (uint32_t b = 0; b < LH2_SYNTH_CAPTURE_BYTES; b += 4) {
            uint32_t r = lh2_synth_rand(&rng);
            memcpy(&spi[b], &r, 4);
        }
        // A running decoder, as on the robot: the stamps advance 1 ms a capture
        uint32_t         t = 1000000U + i * 1000U;
        lh2_ref_result_t r = lh2_ref_feed(&_lh2, spi, t, t + 20);
        sane               = sane && r <= LH2_REF_EQUAL_COUNTS;
        results[r <= LH2_REF_EQUAL_COUNTS ? r : 7]++;
    }
    for (uint8_t station = 0; station < LH2_REF_STATIONS; station++) {
        for (uint8_t sweep = 0; sweep < 2; sweep++) {
            if (_lh2.data_ready[sweep][station] == LH2_REF_PROCESSED_DATA_AVAILABLE) {
                sane = sane && _lh2.locations[sweep][station].lfsr_counts <= _revolution(station) && (_lh2.locations[sweep][station].selected_polynomial >> 1) == station;
            }
        }
    }
    uint32_t accepted = n - results[LH2_REF_MOCAP] - results[LH2_REF_NO_POLYNOMIAL];
    MEASURED("D10 random: %u captures, %u accepted as some polynomial (%u stored), %u mocap", n, accepted, results[LH2_REF_STORED], results[LH2_REF_MOCAP]);
    CHECK(sane, "D10: every result in the enum, every stored count under a revolution and of its station");
    CHECK(accepted <= n / 5000, "D10: random bytes are rarely accepted as a polynomial, %u in %u", accepted, n);
}

static void test_d11_checkpoint_table_ends_at_108325(void) {
    uint32_t last = 0;
    for (uint8_t p = 0; p < LH2_REF_POLYNOMIALS; p++) {
        for (uint8_t slot = 0; slot < LH2_REF_CHECKPOINTS; slot++) {
            uint32_t c = lh2_ref_static_checkpoint(p, slot);
            last       = (c > last) ? c : last;
        }
    }
    // Defect pinned: a channel-1 revolution has 119,875 counts; past 108,325 every search is cold
    CHECK(last == 108325, "D11 (defect): the static checkpoint tables end at count 108,325, got %u", last);

    uint8_t  polys[LH2_REF_POLYNOMIALS];
    uint8_t  n_polys = 0;
    uint32_t wrong = 0, worst_in = 0, worst_past = 0;
    for (uint8_t p = 0; p < LH2_REF_POLYNOMIALS; p++) {
        if (_long || p < 2) {
            polys[n_polys++] = p;
        }
    }
    for (uint8_t i = 0; i < n_polys; i++) {
        uint8_t  p     = polys[i];
        uint32_t rev   = _revolution(p >> 1);
        uint32_t state = 1;
        for (uint32_t n = 0; n < LH2_SYNTH_LFSR_PERIOD; n++) {
            lh2_ref_checkpoint_t cold = { 0 };
            wrong += lh2_ref_lfsr_index_search(&cold, p, state) != n;
            if (n < rev) {
                // The search walks both ways until it meets a checkpoint
                uint32_t steps = LH2_SYNTH_LFSR_PERIOD;
                for (uint8_t slot = 0; slot < LH2_REF_CHECKPOINTS; slot++) {
                    uint32_t c    = lh2_ref_static_checkpoint(p, slot);
                    uint32_t down = (n + LH2_SYNTH_LFSR_PERIOD - c) % LH2_SYNTH_LFSR_PERIOD;
                    uint32_t up   = (c + LH2_SYNTH_LFSR_PERIOD - n) % LH2_SYNTH_LFSR_PERIOD;
                    steps         = (down < steps) ? down : steps;
                    steps         = (up < steps) ? up : steps;
                }
                if (n > last) {
                    worst_past = (steps > worst_past) ? steps : worst_past;
                } else {
                    worst_in = (steps > worst_in) ? steps : worst_in;
                }
            }
            state = ((state << 1) | (__builtin_popcount(state & lh2_ref_polynomial(p)) & 1U)) & 0x1FFFFU;
        }
    }
    MEASURED("D11 cold LFSR search, %u polynomials, every index: %u wrong; most steps %u up to count 108,325, %u past it", n_polys, wrong, worst_in, worst_past);
    CHECK(wrong == 0, "D11: the cold search finds the exact index of every state, %u wrong", wrong);
    CHECK(worst_in < 2100, "D11: inside the tables a search meets a checkpoint within about 2k steps, %u", worst_in);
    CHECK(worst_past > 11000 && worst_past < 11600, "D11 (defect): past count 108,325 a cold search takes up to about 11.4k steps, %u", worst_past);

    // With a dynamic checkpoint near, the search is short and still exact
    uint32_t dyn_wrong = 0;
    for (uint32_t m = 1000; m < 119000; m += 9000) {
        lh2_ref_checkpoint_t dyn = { 0 };
        dyn.bits[3][1]           = lh2_synth_lfsr_state(3, m);
        dyn.count[3][1]          = m;
        for (uint32_t n = m - 500; n <= m + 500; n += 7) {
            dyn_wrong += lh2_ref_lfsr_index_search(&dyn, 3, lh2_synth_lfsr_state(3, n)) != n;
        }
    }
    CHECK(dyn_wrong == 0, "D11: exact with dynamic checkpoints, %u wrong", dyn_wrong);
}

static void test_d12_past_revolution(void) {
    lh2_synth_rng_t rng     = { 2012 };
    uint8_t         station = 15;
    uint32_t        rev     = _revolution(station);
    uint32_t        right = 0, cases = 0;
    for (uint32_t n0 = rev - 2; n0 <= rev + 1; n0++) {
        for (uint32_t step = 0; step < 86; step++) {
            lh2_synth_light_t light = { .polynomial = 30, .count = n0, .phase = step / 16.0, .level = (uint8_t)(step & 1U) };
            uint8_t           spi[LH2_SYNTH_CAPTURE_BYTES];
            lh2_synth_capture(&light, &rng, spi);
            uint8_t  p;
            uint32_t count = _chain_count(spi, &p);
            if (p != 30) {
                continue;
            }
            // A slot already holding this sweep, so the rejection has something to clear
            lh2_ref_init(&_lh2);
            _lh2.data_ready[0][station]            = LH2_REF_PROCESSED_DATA_AVAILABLE;
            _lh2.timestamps[0][station]            = T_US - (uint32_t)lh2_synth_period_us(station);
            _lh2.locations[0][station].lfsr_counts = 50000;
            lh2_ref_result_t r                     = lh2_ref_feed(&_lh2, spi, T_US, T_US + 50);
            cases++;
            if (count <= rev) {
                right += r == LH2_REF_STORED;
            } else {
                right += r == LH2_REF_PAST_REVOLUTION && _lh2.data_ready[0][station] == LH2_REF_NO_NEW_DATA;
            }
        }
    }
    CHECK(cases > 300 && right == cases, "D12: a count up to the revolution's (%u) is stored, past it rejected and its slot cleared: %u of %u", rev, right, cases);
}

static void test_d13_equal_counts(void) {
    lh2_synth_rng_t   rng   = { 2013 };
    lh2_synth_light_t light = { .polynomial = 6, .count = 41000, .phase = 1.0, .level = 0 };
    uint8_t           spi[LH2_SYNTH_CAPTURE_BYTES];
    lh2_synth_capture(&light, &rng, spi);
    lh2_ref_init(&_lh2);
    CHECK(lh2_ref_feed(&_lh2, spi, T_US, T_US + 50) == LH2_REF_STORED, "D13: the first capture is stored");
    // 6 ms later: too far from slot 0's grid, so it takes slot 1, with the same count
    lh2_ref_result_t r = lh2_ref_feed(&_lh2, spi, T_US + 6000, T_US + 6050);
    CHECK(r == LH2_REF_EQUAL_COUNTS, "D13: a count equal to the other sweep's is rejected, got %d", r);
    CHECK(_lh2.data_ready[0][3] == LH2_REF_NO_NEW_DATA && _lh2.data_ready[1][3] == LH2_REF_NO_NEW_DATA, "D13: both slots cleared");
    CHECK(_lh2.timestamps[0][3] == T_US && _lh2.timestamps[1][3] == 0, "D13: slot 0 keeps its stamp, slot 1 is not written");
}

//=========================== drv/lh2_decoder: sweep slots and time ============

/// Feed a stream, taking every station's pair every read_us; counts the pairs that are not the truth
typedef struct {
    uint32_t reads;
    uint32_t pairs;
    uint32_t wrong_pairs;
    uint32_t swapped_pairs;
} run_t;

static run_t _run_stream(const lh2_synth_station_t *stations, uint8_t n_stations, size_t n, double read_us, uint32_t *stored_slot) {
    run_t  run       = { 0 };
    double next_read = read_us;
    lh2_ref_init(&_lh2);
    for (size_t i = 0; i <= n; i++) {
        double t = (i < n) ? _events[i].t_true_us : next_read;
        while (next_read <= t) {
            run.reads++;
            for (uint8_t s = 0; s < n_stations; s++) {
                uint32_t counts[2];
                if (!_take_pair(&_lh2, stations[s].station, counts)) {
                    continue;
                }
                run.pairs++;
                bool straight = _near(counts[0], stations[s].counts[0]) && _near(counts[1], stations[s].counts[1]);
                bool swapped  = _near(counts[0], stations[s].counts[1]) && _near(counts[1], stations[s].counts[0]);
                run.wrong_pairs += !straight && !swapped;
                run.swapped_pairs += swapped;
            }
            next_read += read_us;
        }
        if (i == n) {
            break;
        }
        lh2_ref_result_t r = lh2_ref_feed(&_lh2, _events[i].spi, _events[i].t_us, _events[i].t_us + 20);
        if (stored_slot) {
            stored_slot[i] = (r == LH2_REF_STORED) ? (uint32_t)_slot_of(&_lh2, &_events[i]) : 0xFFU;
        }
    }
    return run;
}

static void test_s1_one_station(void) {
    lh2_synth_rng_t     rng = { 3001 };
    lh2_synth_station_t st  = { .station = 1, .counts = { 30000, 70000 }, .t0_us = 1000 };
    size_t              n   = lh2_synth_stream(&st, 1, 1000 * lh2_synth_period_us(1), 1000000, &rng, _events, 40000);
    uint32_t            slots[2100];
    run_t               run       = _run_stream(&st, 1, n, 100000, slots);
    uint32_t            misplaced = 0;
    for (size_t i = 0; i < n; i++) {
        misplaced += slots[i] != 0xFFU && slots[i] != _events[i].sweep;
    }
    CHECK(misplaced == 0, "S1: the first sweep takes slot 0 and every later capture its sweep's slot, %u misplaced", misplaced);
    CHECK(run.pairs >= run.reads - 1 && run.wrong_pairs == 0 && run.swapped_pairs == 0, "S1: every read gives the true pair, %u of %u reads", run.pairs, run.reads);

    st.first_drop = 1;  // the station's first capture is its second sweep
    n             = lh2_synth_stream(&st, 1, 1000 * lh2_synth_period_us(1), 1000000, &rng, _events, 40000);
    run           = _run_stream(&st, 1, n, 100000, NULL);
    CHECK(run.swapped_pairs == run.pairs && run.pairs > 0, "S1: from a cold start on the second sweep, the pair is swapped for good: %u of %u", run.swapped_pairs, run.pairs);
}

static void test_s2_late_duplicate(void) {
    // A second capture of one pass lands in the same slot and replaces its count
    lh2_synth_rng_t     rng = { 3002 };
    lh2_synth_station_t st  = { .station = 1, .counts = { 30000, 70000 }, .t0_us = 1000, .dup_share = 0.3, .dup_delay_us = { 20, 60 }, .dup_sweeps = 1 };
    size_t              n   = lh2_synth_stream(&st, 1, 2000 * lh2_synth_period_us(1), 1000000, &rng, _events, 40000);
    lh2_ref_init(&_lh2);
    uint32_t direct = 0, duplicate = 0, other = 0, reads = 0;
    double   next = 100000;
    for (size_t i = 0; i < n; i++) {
        while (next <= _events[i].t_true_us) {
            uint32_t counts[2];
            if (_take_pair(&_lh2, 1, counts)) {
                reads++;
                if (_near(counts[0], 30000)) {
                    direct++;
                } else if (counts[0] >= 30000 + 120 && counts[0] <= 30000 + 361) {
                    duplicate++;
                } else {
                    other++;
                }
                other += !_near(counts[1], 70000);
            }
            next += 100000;
        }
        lh2_ref_feed(&_lh2, _events[i].spi, _events[i].t_us, _events[i].t_us + 20);
    }
    MEASURED("S2 late duplicate, 30%% of sweep-0 passes, 20-60 us late: %u reads, sweep 0 read the direct count %u times, the duplicate's %u, other %u", reads, direct, duplicate, other);
    // Defect pinned: the read carries the duplicate's count as often as the duplicate is the pass's last capture
    CHECK(duplicate > reads / 5 && duplicate < reads * 2 / 5, "S2 (defect): the duplicate replaces the slot, read back in about 30%% of reads: %u of %u", duplicate, reads);
    CHECK(other == 0, "S2: no other count is ever read");

    // One pass, the duplicate 33 us late: about +198 counts, +20 mm at bench range
    lh2_synth_light_t a = { .polynomial = 2, .count = 30000, .phase = 2.0 }, b = { .polynomial = 2, .count = 70000, .phase = 2.0 }, dup = { .polynomial = 2, .count = 30198, .phase = 2.0 };
    uint8_t           sa[LH2_SYNTH_CAPTURE_BYTES], sb[LH2_SYNTH_CAPTURE_BYTES], sd[LH2_SYNTH_CAPTURE_BYTES];
    lh2_synth_capture(&a, &rng, sa);
    lh2_synth_capture(&b, &rng, sb);
    lh2_synth_capture(&dup, &rng, sd);
    lh2_ref_init(&_lh2);
    lh2_ref_feed(&_lh2, sa, T_US + 5000, T_US + 5020);
    lh2_ref_feed(&_lh2, sd, T_US + 5033, T_US + 5053);
    lh2_ref_feed(&_lh2, sb, T_US + 11667, T_US + 11687);
    uint32_t counts[2] = { 0 };
    CHECK(_take_pair(&_lh2, 1, counts) && _near(counts[0], 30198) && _near(counts[1], 70000), "S2 (defect): the pair read after a 33 us duplicate carries the duplicate's count, %u", counts[0]);
}

static void test_s3_silent_station(void) {
    lh2_synth_rng_t     rng = { 3003 };
    lh2_synth_station_t st  = { .station = 4, .counts = { 25000, 61000 }, .t0_us = 1000, .silent_us = { 3000000, 5500000 } };
    size_t              n   = lh2_synth_stream(&st, 1, 8000000, 1000000, &rng, _events, 40000);
    lh2_ref_init(&_lh2);
    bool     cleared = false;
    uint32_t back = 0, back_pairs = 0, back_wrong = 0;
    uint32_t back_first_sweep = 2;
    for (size_t i = 0; i < n; i++) {
        bool first_back = _events[i].t_true_us >= 5500000 && back == 0;
        lh2_ref_feed(&_lh2, _events[i].spi, _events[i].t_us, _events[i].t_us + 20);
        if (first_back) {
            back             = 1;
            back_first_sweep = _events[i].sweep;
            // The stale slots were cleared, and the returning capture took slot 0
            cleared = _slot_of(&_lh2, &_events[i]) == 0 && _lh2.timestamps[1][4] == 0;
        }
        if (back && _events[i].sweep == 1) {
            uint32_t counts[2];
            if (_take_pair(&_lh2, 4, counts)) {
                back_pairs++;
                bool straight = _near(counts[0], 25000) && _near(counts[1], 61000);
                bool swapped  = _near(counts[0], 61000) && _near(counts[1], 25000);
                back_wrong += !(back_first_sweep == 0 ? straight : swapped);
            }
        }
    }
    CHECK(cleared, "S3: after 2.5 s of silence both slots are stale, cleared, and the first capture back takes slot 0");
    CHECK(back_pairs > 50 && back_wrong == 0, "S3: re-acquired, the pair keeps the order of the first capture back: %u wrong of %u", back_wrong, back_pairs);
    double straight[2], swapped[2];
    db_lh2_camera_point(25000, 61000, 4, straight);
    db_lh2_camera_point(61000, 25000, 4, swapped);
    CHECK(straight[0] == swapped[0] && straight[1] == swapped[1], "S3: the camera point does not depend on the slot order");
}

static void test_s4_timer_wrap(void) {
    lh2_synth_station_t sts[2] = {
        { .station = 2, .counts = { 22000, 64000 }, .t0_us = 1000 },
        { .station = 9, .counts = { 35000, 81000 }, .t0_us = 7300, .silent_us = { 2000000, 4600000 } },
    };
    static uint32_t base_slots[40000], wrap_slots[40000];
    lh2_synth_rng_t rng  = { 3004 };
    size_t          n    = lh2_synth_stream(sts, 2, 8000000, 1000000, &rng, _events, 40000);
    run_t           base = _run_stream(sts, 2, n, 100000, base_slots);
    // The same run with 2^32 inside station 9's silent gap and between sweeps elsewhere
    rng         = (lh2_synth_rng_t){ 3004 };
    size_t n2   = lh2_synth_stream(sts, 2, 8000000, 0x100000000ULL - 3000000ULL, &rng, _events, 40000);
    bool   zero = false;
    for (size_t i = 0; i < n2; i++) {
        zero = zero || _events[i].t_us == 0;
    }
    run_t    wrap   = _run_stream(sts, 2, n2, 100000, wrap_slots);
    uint32_t differ = 0;
    for (size_t i = 0; i < n && i < n2; i++) {
        differ += base_slots[i] != wrap_slots[i];
    }
    CHECK(!zero, "S4: no stamp falls on exactly 0 in this run");
    CHECK(n == n2 && differ == 0 && base.pairs == wrap.pairs && wrap.wrong_pairs == 0, "S4: the 2^32 us wrap changes no slot and no pair: %u differ", differ);
}

static void test_s5_stamp_zero(void) {
    lh2_synth_rng_t   rng = { 3005 };
    lh2_synth_light_t a = { .polynomial = 10, .count = 30000, .phase = 1.0 }, b = { .polynomial = 10, .count = 70000, .phase = 1.0 };
    uint8_t           sa[LH2_SYNTH_CAPTURE_BYTES], sb[LH2_SYNTH_CAPTURE_BYTES];
    lh2_synth_capture(&a, &rng, sa);
    lh2_synth_capture(&b, &rng, sb);
    lh2_ref_init(&_lh2);
    CHECK(lh2_ref_feed(&_lh2, sa, 0, 20) == LH2_REF_STORED, "S5: a capture stamped 0 is stored");
    lh2_ref_feed(&_lh2, sb, 6667, 6687);
    // Defect pinned: a stamp of 0 marks the slot empty, so the other sweep lands on it
    CHECK(_lh2.data_ready[1][5] != LH2_REF_PROCESSED_DATA_AVAILABLE && _near(_lh2.locations[0][5].lfsr_counts, 70000), "S5 (defect): the slot stamped 0 reads as empty, the next sweep overwrites it");
}

static void test_s6_handler_latency(void) {
    const double latencies[] = { 0, 500, 900, 1500, 2000, 3000 };
    uint32_t     wrong[6];
    uint32_t     trials = 400;
    for (uint32_t l = 0; l < 6; l++) {
        lh2_synth_rng_t rng = { 3006 };
        wrong[l]            = 0;
        for (uint32_t t = 0; t < trials; t++) {
            // A cold start that sees sweep 0 twice before sweep 1, so one slot is empty
            lh2_synth_station_t st = { .station = 1, .counts = { 30000, 70000 }, .t0_us = 1000, .first_drop = 2, .latency_us = latencies[l] };
            size_t              n  = lh2_synth_stream(&st, 1, 1000 + 2 * lh2_synth_period_us(1) - 1, 1000000, &rng, _events, 40000);
            uint32_t            slots[8];
            _run_stream(&st, 1, n, 1e9, slots);
            int  sweep0 = -1, sweep1 = -1;
            bool bad = false;
            for (size_t i = 0; i < n; i++) {
                int *seen = _events[i].sweep ? &sweep1 : &sweep0;
                if (slots[i] == 0xFFU) {
                    continue;
                }
                bad   = bad || (*seen >= 0 && *seen != (int)slots[i]);
                *seen = (int)slots[i];
            }
            wrong[l] += bad || (sweep0 >= 0 && sweep0 == sweep1);
        }
    }
    MEASURED("S6 handler latency 0 / 0.5 / 0.9 / 1.5 / 2 / 3 ms, one slot empty: %u / %u / %u / %u / %u / %u of %u cold starts misassigned", wrong[0], wrong[1], wrong[2], wrong[3], wrong[4], wrong[5], trials);
    CHECK(wrong[0] == 0 && wrong[1] == 0 && wrong[2] == 0, "S6: under 1 ms of latency no slot is misassigned");
    // Defect pinned: past 1 ms, a capture beside an empty slot misses its own grid and takes the empty one
    CHECK(wrong[3] > 0 && wrong[5] > wrong[3], "S6 (defect): past 1 ms of latency a cold start misassigns slots, more with more latency");

    // With both slots full the nearer grid wins, so 3 ms is harmless when the sweeps are 6.7 ms apart
    lh2_synth_rng_t     rng  = { 3007 };
    lh2_synth_station_t st   = { .station = 1, .counts = { 30000, 70000 }, .t0_us = 1000, .latency_us = 3000 };
    size_t              n    = lh2_synth_stream(&st, 1, 400 * lh2_synth_period_us(1), 1000000, &rng, _events, 40000);
    run_t               full = _run_stream(&st, 1, n, 100000, NULL);
    CHECK(full.wrong_pairs == 0 && full.swapped_pairs == 0, "S6: with both slots held, 3 ms of latency misassigns nothing");
}

static void test_s7_stations_interleaved(void) {
    lh2_synth_rng_t     rng    = { 3008 };
    lh2_synth_station_t sts[4] = {
        { .station = 0, .counts = { 31000, 72000 }, .t0_us = 100 },
        { .station = 3, .counts = { 28000, 66000 }, .t0_us = 4100 },
        { .station = 7, .counts = { 40000, 83000 }, .t0_us = 9300 },
        { .station = 12, .counts = { 26000, 69000 }, .t0_us = 15700 },
    };
    size_t n   = lh2_synth_stream(sts, 4, 30000000, 1000000, &rng, _events, 40000);
    run_t  run = _run_stream(sts, 4, n, 100000, NULL);
    CHECK(run.pairs > 4 * (run.reads - 2) && run.wrong_pairs == 0 && run.swapped_pairs == 0, "S7: four stations on four channels keep their own slots, %u pairs, %u wrong", run.pairs, run.wrong_pairs);
}

static void test_s8_shared_channel(void) {
    // Two stations on one channel: the decoder sees four sweeps of one station
    lh2_synth_rng_t     rng    = { 3009 };
    lh2_synth_station_t sts[2] = {
        { .station = 5, .counts = { 30000, 70000 }, .t0_us = 1000 },
        { .station = 5, .counts = { 45000, 95000 }, .t0_us = 5000 },
    };
    size_t   n     = lh2_synth_stream(sts, 2, 40000000, 1000000, &rng, _events, 40000);
    uint32_t reads = 0, pure = 0, mixed = 0;
    double   next = 100000;
    lh2_ref_init(&_lh2);
    for (size_t i = 0; i < n; i++) {
        while (next <= _events[i].t_true_us) {
            uint32_t counts[2];
            if (_take_pair(&_lh2, 5, counts)) {
                reads++;
                int owner[2];
                for (int k = 0; k < 2; k++) {
                    owner[k] = (_near(counts[k], 30000) || _near(counts[k], 70000)) ? 0 : ((_near(counts[k], 45000) || _near(counts[k], 95000)) ? 1 : 2);
                }
                bool same = owner[0] == owner[1] && owner[0] != 2 && counts[0] != counts[1];
                pure += same;
                mixed += !same;
            }
            next += 100000;
        }
        lh2_ref_feed(&_lh2, _events[i].spi, _events[i].t_us, _events[i].t_us + 20);
    }
    MEASURED("S8 two stations on one channel: %u reads, %u one station's pair, %u mixed", reads, pure, mixed);
    // Defect pinned: nothing tells two stations of one channel apart, so pairs mix them
    CHECK(mixed > reads / 10, "S8 (defect): pairs from a shared channel mix the two stations, %u of %u", mixed, reads);
}

int main(int argc, char **argv) {
    _long = argc > 1 && strcmp(argv[1], "--long") == 0;
    test_synth_polynomials_and_lfsr();
    test_synth_code();
    test_synth_knobs();
    test_synth_stream();
    test_d1_clean();
    test_d2_start_phase();
    test_d3_sample_flips();
    test_d3_dead_tolerance();
    test_d4_confuse();
    test_d5_jitter();
    test_d6_mocap_filter_on_by_default();
    test_d7_mocap_strobe();
    test_d8_collide();
    test_d9_constant_captures();
    test_d10_random_captures();
    test_d11_checkpoint_table_ends_at_108325();
    test_d12_past_revolution();
    test_d13_equal_counts();
    test_s1_one_station();
    test_s2_late_duplicate();
    test_s3_silent_station();
    test_s4_timer_wrap();
    test_s5_stamp_zero();
    test_s6_handler_latency();
    test_s7_stations_interleaved();
    test_s8_shared_channel();
    printf("lh2_baseline%s: %d passed, %d failed\n", _long ? " (long)" : "", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
