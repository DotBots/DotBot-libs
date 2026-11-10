/**
 * @file
 * @brief  Host test of drv/lh2_decoder against the frozen 228ce9a decode path
 *
 * The golden comparison: every capture goes to both the reference in
 * lh2_reference.c and the decoder, and after each one the result, all 32
 * sweep slots and the dynamic checkpoints must be identical. A difference
 * prints the input set, the capture's index and seed and the capture in hex,
 * so the case replays on its own. `--long` runs the full-size sweeps.
 *
 * @copyright Inria, 2026
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lh2_decoder.h"
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

#define MEASURED(...)              \
    do {                           \
        printf("  lh2_decoder: "); \
        printf(__VA_ARGS__);       \
        printf("\n");              \
    } while (0)

_Static_assert(DB_LH2_CAPTURE_BYTES == LH2_REF_CAPTURE_BYTES && DB_LH2_CAPTURE_BYTES == LH2_SYNTH_CAPTURE_BYTES, "one capture size");
_Static_assert(DB_LH2_STATIONS == LH2_REF_STATIONS && DB_LH2_POLYNOMIALS == LH2_REF_POLYNOMIALS && DB_LH2_SWEEPS == LH2_REF_SWEEPS, "one slot layout");
_Static_assert(DB_LH2_CHECKPOINTS == LH2_REF_CHECKPOINTS, "one checkpoint count");

static lh2_ref_t         _ref;
static db_lh2_decoder_t  _dec;
static lh2_synth_event_t _events[40000];

/// The golden comparison's tally across every input set
static uint32_t _compared = 0;
static uint32_t _differ   = 0;
static uint32_t _ref_results[DB_LH2_DECODE_RESULTS];
static uint32_t _printed = 0;

static void _hex(const char *what, const uint8_t spi[DB_LH2_CAPTURE_BYTES]) {
    printf("  %s:", what);
    for (uint32_t i = 0; i < DB_LH2_CAPTURE_BYTES; i++) {
        printf("%02x", spi[i]);
    }
    printf("\n");
}

//=========================== golden comparison ================================

static db_lh2_decode_result_t _from_ref(lh2_ref_result_t r) {
    switch (r) {
        case LH2_REF_STORED:
            return DB_LH2_DECODE_STORED;
        case LH2_REF_MOCAP:
            return DB_LH2_DECODE_MOCAP;
        case LH2_REF_NO_POLYNOMIAL:
            return DB_LH2_DECODE_NO_POLYNOMIAL;
        case LH2_REF_ZERO_BITS:
            return DB_LH2_DECODE_ZERO_BITS;
        case LH2_REF_NO_INDEX:
            return DB_LH2_DECODE_NO_INDEX;
        case LH2_REF_PAST_REVOLUTION:
            return DB_LH2_DECODE_PAST_REVOLUTION;
        case LH2_REF_EQUAL_COUNTS:
            return DB_LH2_DECODE_EQUAL_COUNTS;
    }
    return DB_LH2_DECODE_RESULTS;
}

/// The first field where the decoder and the reference disagree, or NULL
static const char *_state_differs(char *where, size_t len) {
    for (uint8_t sweep = 0; sweep < DB_LH2_SWEEPS; sweep++) {
        for (uint8_t station = 0; station < DB_LH2_STATIONS; station++) {
            const db_lh2_slot_t *s = &_dec.slot[sweep][station];
            bool                 ready_ref;
            if (_ref.data_ready[sweep][station] == LH2_REF_PROCESSED_DATA_AVAILABLE) {
                ready_ref = true;
            } else if (_ref.data_ready[sweep][station] == LH2_REF_NO_NEW_DATA) {
                ready_ref = false;
            } else {
                snprintf(where, len, "reference slot %u/%u in a state the decoder has no word for", sweep, station);
                return where;
            }
            if (s->count != _ref.locations[sweep][station].lfsr_counts || (uint32_t)s->polynomial != _ref.locations[sweep][station].selected_polynomial || s->t_us != _ref.timestamps[sweep][station] || s->ready != ready_ref || (_ref.timestamps[sweep][station] != 0 && !s->filled)) {
                snprintf(where, len, "slot %u/%u: decoder count %u poly %u t %u ready %d, reference %u %u %u %d", sweep, station, s->count, s->polynomial, s->t_us, s->ready, _ref.locations[sweep][station].lfsr_counts, _ref.locations[sweep][station].selected_polynomial, _ref.timestamps[sweep][station], ready_ref);
                return where;
            }
        }
    }
    const lh2_ref_checkpoint_t *cp = lh2_ref_checkpoints();
    for (uint8_t p = 0; p < DB_LH2_POLYNOMIALS; p++) {
        for (uint8_t sweep = 0; sweep < DB_LH2_SWEEPS; sweep++) {
            if (_dec.checkpoint_bits[p][sweep] != cp->bits[p][sweep] || _dec.checkpoint_count[p][sweep] != cp->count[p][sweep]) {
                snprintf(where, len, "dynamic checkpoint %u/%u", p, sweep);
                return where;
            }
        }
    }
    return NULL;
}

static void _both_init(void) {
    lh2_ref_init(&_ref);
    db_lh2_decoder_init(&_dec);
}

/// The capture's station holds a capture stamped exactly 0, which the frozen copy reads as empty (row S5)
static bool _s5_pending(const uint8_t spi[DB_LH2_CAPTURE_BYTES]) {
    int8_t  offset;
    uint8_t polynomial = db_lh2_polynomial(db_lh2_demodulate(spi), &offset);
    if (polynomial == DB_LH2_POLYNOMIAL_NONE) {
        return false;
    }
    for (uint8_t sweep = 0; sweep < DB_LH2_SWEEPS; sweep++) {
        const db_lh2_slot_t *slot = &_dec.slot[sweep][polynomial >> 1];
        if (slot->filled && slot->t_us == 0) {
            return true;
        }
    }
    return false;
}

/// The decoder's slots and checkpoints set to the frozen copy's, its counters kept
static void _resync(void) {
    for (uint8_t sweep = 0; sweep < DB_LH2_SWEEPS; sweep++) {
        for (uint8_t station = 0; station < DB_LH2_STATIONS; station++) {
            db_lh2_slot_t *s = &_dec.slot[sweep][station];
            s->count         = _ref.locations[sweep][station].lfsr_counts;
            s->polynomial    = (uint8_t)_ref.locations[sweep][station].selected_polynomial;
            s->t_us          = _ref.timestamps[sweep][station];
            s->ready         = _ref.data_ready[sweep][station] == LH2_REF_PROCESSED_DATA_AVAILABLE;
            s->filled        = s->t_us != 0 || s->ready;
        }
    }
    const lh2_ref_checkpoint_t *cp = lh2_ref_checkpoints();
    memcpy(_dec.checkpoint_bits, cp->bits, sizeof(cp->bits));
    memcpy(_dec.checkpoint_count, cp->count, sizeof(cp->count));
}

static uint32_t _declared_s5 = 0;

/// Inputs in each class of declared divergence, and those whose outcome was not the declared one
#if !defined(LH2_MOCAP_FILTER)
static uint32_t _declared_mocap      = 0;
static uint32_t _declared_mocap_kept = 0;  ///< of them, still rejected as motion capture
#endif
static uint32_t _declared_wrong       = 0;
static uint32_t _declared_weak        = 0;
static uint32_t _declared_weak_stored = 0;  ///< of them, stored by the frozen copy

/// One capture into both; false, and the case printed, on any difference.
/// Declared divergences: without LH2_MOCAP_FILTER, a capture the frozen copy
/// rejects as motion capture is decoded like any other (the decoder's state
/// is then put back, as the copy's never changed); and while a slot holds a
/// capture stamped 0, which the copy takes for empty, or the capture itself is
/// stamped 0, it is fed to both and the decoder then set to the copy's state
/// (row S5).
/// A weak match the decoder does not confirm is the third class: the decoder
/// keeps its slots, and is then set to the copy's state.
static bool _feed_both(const char *set, uint32_t index, uint32_t seed, const uint8_t spi[DB_LH2_CAPTURE_BYTES], uint32_t t_us, uint32_t now_us) {
    db_lh2_capture_t cap;
    memcpy(cap.spi, spi, DB_LH2_CAPTURE_BYTES);
    cap.t_us                    = t_us;
    db_lh2_decode_result_t want = _from_ref(lh2_ref_feed(&_ref, spi, t_us, now_us));
#if !defined(LH2_MOCAP_FILTER)
    if (want == DB_LH2_DECODE_MOCAP) {
        db_lh2_decoder_t before = _dec;
        _declared_mocap++;
        _declared_mocap_kept += db_lh2_decoder_feed(&_dec, &cap, now_us) == DB_LH2_DECODE_MOCAP;
        _dec = before;
        _compared++;
        _ref_results[want]++;
        return true;
    }
#endif
    if (t_us == 0 || _s5_pending(spi)) {
        _declared_s5++;
        db_lh2_decoder_feed(&_dec, &cap, now_us);
        _resync();
        _compared++;
        _ref_results[want]++;
        return true;
    }
    uint32_t               counted = _dec.results[want < DB_LH2_DECODE_RESULTS ? want : 0];
    db_lh2_decode_result_t got     = db_lh2_decoder_feed(&_dec, &cap, now_us);
    if (got == DB_LH2_DECODE_UNCONFIRMED) {
        // declared: a weak match away from its station's last count is not kept
        int8_t  offset;
        uint8_t compared;
        db_lh2_polynomial_match(db_lh2_demodulate(spi), &offset, &compared);
        _declared_weak++;
        _declared_wrong += compared >= DB_LH2_WEAK_MATCH_BITS;
        _declared_weak_stored += want == DB_LH2_DECODE_STORED;
        _resync();
        _compared++;
        _ref_results[want]++;
        return true;
    }
    char        buf[200];
    const char *differ = (got != want) ? "the result" : _state_differs(buf, sizeof(buf));
    if (differ == NULL && _dec.results[want] != counted + 1U) {
        differ = "the result counter";
    }
    _compared++;
    if (want < DB_LH2_DECODE_RESULTS) {
        _ref_results[want]++;
    }
    if (differ == NULL) {
        return true;
    }
    _differ++;
    if (_printed++ < 5) {
        printf("  DIFFER %s #%u seed %u, t %u now %u: %s (decoder result %d, reference %d)\n", set, index, seed, t_us, now_us, differ, got, want);
        _hex("capture", spi);
    }
    return false;
}

/// The swarmit glue's pair take, on the reference
static bool _ref_take_pair(uint8_t station, uint32_t counts[2]) {
    if (_ref.data_ready[0][station] != LH2_REF_PROCESSED_DATA_AVAILABLE || _ref.data_ready[1][station] != LH2_REF_PROCESSED_DATA_AVAILABLE) {
        return false;
    }
    counts[0]                   = _ref.locations[0][station].lfsr_counts;
    counts[1]                   = _ref.locations[1][station].lfsr_counts;
    _ref.data_ready[0][station] = LH2_REF_NO_NEW_DATA;
    _ref.data_ready[1][station] = LH2_REF_NO_NEW_DATA;
    return true;
}

//=========================== inputs ===========================================

static uint32_t _revolution(uint8_t station) {
    return db_lh2_period(station) / DB_LH2_PERIOD_TICKS_PER_COUNT;
}

static lh2_synth_light_t _light(lh2_synth_rng_t *rng, uint8_t polynomial, uint32_t count) {
    lh2_synth_light_t light = { 0 };
    light.polynomial        = polynomial;
    light.count             = count;
    light.phase             = lh2_synth_uniform(rng) * LH2_SYNTH_SAMPLES_PER_BIT;
    light.level             = (uint8_t)(lh2_synth_rand(rng) >> 31);
    return light;
}

static lh2_synth_light_t _random_light(lh2_synth_rng_t *rng) {
    uint8_t polynomial = (uint8_t)(lh2_synth_rand(rng) >> 27);
    return _light(rng, polynomial, (uint32_t)(lh2_synth_uniform(rng) * _revolution(polynomial >> 1)));
}

/// A count of row D1's classes: 0, 17, a static checkpoint +-1, past the last one, the revolution's end, anywhere
static uint32_t _position(lh2_synth_rng_t *rng, uint8_t polynomial) {
    uint32_t revolution = _revolution(polynomial >> 1);
    switch (lh2_synth_rand(rng) % 7) {
        case 0:
            return 0;
        case 1:
            return 17;
        case 2:
        {
            uint32_t at = lh2_ref_static_checkpoint(polynomial, (uint8_t)(lh2_synth_rand(rng) % LH2_REF_CHECKPOINTS));
            int32_t  d  = (int32_t)(lh2_synth_rand(rng) % 3) - 1;
            return (at == 0 && d < 0) ? 0 : (uint32_t)((int32_t)at + d);
        }
        case 3:
            return 108326U + (uint32_t)(lh2_synth_uniform(rng) * (revolution - 108326U));
        case 4:
            return revolution - 1U - (lh2_synth_rand(rng) % 3);
        default:
            return (uint32_t)(lh2_synth_uniform(rng) * revolution);
    }
}

/// The stamp of the next capture: mostly a few ms on, sometimes past the 2 s staleness, sometimes across 2^32
static uint32_t _next_t(lh2_synth_rng_t *rng, uint32_t t) {
    uint32_t r = lh2_synth_rand(rng) % 100;
    if (r < 3) {
        return t + 2000000U + (lh2_synth_rand(rng) % 2000U);
    }
    if (r < 5) {
        return 0xFFFFFFFFU - (lh2_synth_rand(rng) % 40000U);
    }
    if (r < 6) {
        return 0;
    }
    return t + 1U + (lh2_synth_rand(rng) % 25000U);
}

static uint32_t _now(lh2_synth_rng_t *rng, uint32_t t) {
    return (lh2_synth_rand(rng) % 50 == 0) ? t + 1999990U + (lh2_synth_rand(rng) % 20U) : t + (lh2_synth_rand(rng) % 3000U);
}

//=========================== tests ============================================

static void test_tables(void) {
    uint32_t wrong_poly = 0, wrong_index = 0, wrong_state = 0, wrong_lfsr = 0;
    for (uint8_t p = 0; p < DB_LH2_POLYNOMIALS; p++) {
        wrong_poly += db_lh2_polynomials[p] != lh2_ref_polynomial(p);
        uint32_t last = 0;
        for (uint8_t i = 0; i < DB_LH2_CHECKPOINTS; i++) {
            uint32_t word  = db_lh2_checkpoints[p][i];
            uint32_t index = word >> 11;
            uint32_t state = ((uint32_t)i << 11) | (word & 0x7FFU);
            wrong_index += index != lh2_ref_static_checkpoint(p, i);
            wrong_state += state != lh2_ref_static_checkpoint_state(p, i);
            last = (index > last) ? index : last;
        }
        // every packed entry is the LFSR's state at its index
        uint32_t state = 1;
        for (uint32_t n = 0; n <= last; n++) {
            for (uint8_t i = 0; i < DB_LH2_CHECKPOINTS; i++) {
                if ((db_lh2_checkpoints[p][i] >> 11) == n && state != (((uint32_t)i << 11) | (db_lh2_checkpoints[p][i] & 0x7FFU))) {
                    wrong_lfsr++;
                }
            }
            state = ((state << 1) | (__builtin_popcount(state & db_lh2_polynomials[p]) & 1U)) & 0x1FFFFU;
        }
    }
    CHECK(wrong_poly == 0, "polynomials equal to the reference's: %u differ", wrong_poly);
    CHECK(wrong_index == 0 && wrong_state == 0, "packed checkpoints unpack to the reference's index and state tables: %u indices, %u states differ", wrong_index, wrong_state);
    CHECK(wrong_lfsr == 0, "every packed checkpoint is the LFSR state at its index: %u wrong", wrong_lfsr);
    CHECK(sizeof(db_lh2_checkpoints) == 8192, "the static checkpoints take 8 KiB: %zu B", sizeof(db_lh2_checkpoints));
}

static void test_init_and_reset(void) {
    _both_init();
    char buf[200];
    CHECK(_state_differs(buf, sizeof(buf)) == NULL, "init: decoder and reference agree");
    CHECK(_dec.slot[1][15].polynomial == DB_LH2_POLYNOMIAL_NONE && _dec.slot[1][15].count == DB_LH2_INDEX_NONE && !_dec.slot[1][15].ready, "init: slots empty");

    lh2_synth_rng_t rng = { 4001 };
    uint32_t        t   = 1000000;
    for (uint32_t i = 0; i < 400; i++) {
        lh2_synth_light_t light = _random_light(&rng);
        uint8_t           spi[DB_LH2_CAPTURE_BYTES];
        lh2_synth_capture(&light, &rng, spi);
        _feed_both("reset", i, 4001, spi, t, t + 30);
        t += 3000;
    }
    lh2_ref_reset(&_ref);
    db_lh2_decoder_reset(&_dec);
    CHECK(_state_differs(buf, sizeof(buf)) == NULL, "reset: decoder and reference agree, dynamic checkpoints kept");
    bool any = false;
    for (uint8_t p = 0; p < DB_LH2_POLYNOMIALS; p++) {
        any = any || _dec.checkpoint_bits[p][0] != 0 || _dec.checkpoint_bits[p][1] != 0;
    }
    CHECK(any, "reset keeps the dynamic checkpoints");
}

static void test_low_layer(void) {
    lh2_synth_rng_t rng   = { 4002 };
    uint32_t        n     = _long ? 400000 : 60000;
    uint32_t        demod = 0, poly = 0, mocap = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint8_t spi[DB_LH2_CAPTURE_BYTES];
        if (i % 3 == 0) {
            for (uint32_t b = 0; b < DB_LH2_CAPTURE_BYTES; b++) {
                spi[b] = (uint8_t)(lh2_synth_rand(&rng) >> 24);
            }
        } else {
            lh2_synth_light_t light = _random_light(&rng);
            light.jitter            = (i % 3 == 2) ? lh2_synth_uniform(&rng) : 0.0;
            lh2_synth_capture(&light, &rng, spi);
            if (i % 5 == 0) {
                lh2_synth_flip_samples(spi, 1U + (lh2_synth_rand(&rng) % 12U), &rng);
            }
        }
        uint64_t bits_ref = lh2_ref_demodulate(spi);
        uint64_t bits     = db_lh2_demodulate(spi);
        demod += bits != bits_ref;
        int8_t off_ref = 0, off = 0;
        poly += lh2_ref_determine_polynomial(bits_ref, &off_ref) != db_lh2_polynomial(bits_ref, &off) || off != off_ref;
        mocap += lh2_ref_mocap_interference(spi) != db_lh2_mocap_interference(spi);
    }
    CHECK(demod == 0 && poly == 0 && mocap == 0, "low layer identical on %u captures: demodulation %u, polynomial %u, mocap check %u differ", n, demod, poly, mocap);
}

static void test_lfsr_index_every_position(void) {
    db_lh2_decoder_t     cold     = { 0 };
    lh2_ref_checkpoint_t ref_cold = { 0 };
    uint32_t             differ = 0, searched = 0;
    for (uint8_t p = 0; p < DB_LH2_POLYNOMIALS; p++) {
        if (!_long && p != 0 && p != 19) {
            continue;
        }
        uint32_t state = 1;
        for (uint32_t n = 0; n < LH2_SYNTH_LFSR_PERIOD; n++) {
            differ += db_lh2_lfsr_index(&cold, p, state) != lh2_ref_lfsr_index_search(&ref_cold, p, state);
            searched++;
            state = ((state << 1) | (__builtin_popcount(state & db_lh2_polynomials[p]) & 1U)) & 0x1FFFFU;
        }
    }
    CHECK(differ == 0, "cold LFSR search identical at every index: %u of %u differ", differ, searched);

    // with random dynamic checkpoints, and states that are not on the sequence
    lh2_synth_rng_t rng = { 4003 };
    differ              = 0;
    uint32_t n          = _long ? 200000 : 20000;
    for (uint32_t i = 0; i < n; i++) {
        uint8_t p = (uint8_t)(lh2_synth_rand(&rng) >> 27);
        for (uint8_t s = 0; s < DB_LH2_SWEEPS; s++) {
            uint32_t at                = lh2_synth_rand(&rng) % LH2_SYNTH_LFSR_PERIOD;
            cold.checkpoint_bits[p][s] = ref_cold.bits[p][s] = lh2_synth_lfsr_state(p, at);
            cold.checkpoint_count[p][s] = ref_cold.count[p][s] = (i % 4 == 0) ? lh2_synth_rand(&rng) : at;
        }
        uint32_t bits = (i % 10 == 0) ? (lh2_synth_rand(&rng) & 0x1FFFFU) : lh2_synth_lfsr_state(p, lh2_synth_rand(&rng) % LH2_SYNTH_LFSR_PERIOD);
        differ += db_lh2_lfsr_index(&cold, p, bits) != lh2_ref_lfsr_index_search(&ref_cold, p, bits);
    }
    CHECK(differ == 0, "LFSR search with dynamic checkpoints identical: %u of %u differ", differ, n);
}

static void test_golden_clean(void) {
    lh2_synth_rng_t rng    = { 4004 };
    uint32_t        n      = 20000;
    uint32_t        before = _differ;
    uint32_t        t      = 1000000;
    _both_init();
    for (uint32_t i = 0; i < n; i++) {
        uint8_t           polynomial = (uint8_t)(i % DB_LH2_POLYNOMIALS);
        lh2_synth_light_t light      = _light(&rng, polynomial, _position(&rng, polynomial));
        uint8_t           spi[DB_LH2_CAPTURE_BYTES];
        lh2_synth_capture(&light, &rng, spi);
        if (i % 1000 == 999) {
            _both_init();
        }
        t = _next_t(&rng, t);
        _feed_both("clean", i, 4004, spi, t, _now(&rng, t));
    }
    CHECK(_differ == before, "golden, clean: %u of %u captures differ", _differ - before, n);
}

static void test_golden_impaired(void) {
    lh2_synth_rng_t rng    = { 4005 };
    uint32_t        n      = _long ? 300000 : 100000;
    uint32_t        before = _differ;
    uint32_t        t      = 1000000;
    _both_init();
    for (uint32_t i = 0; i < n; i++) {
        lh2_synth_light_t light = _random_light(&rng);
        uint8_t           spi[DB_LH2_CAPTURE_BYTES];
        switch (i % 8) {
            case 0:  // flips
                lh2_synth_capture(&light, &rng, spi);
                lh2_synth_flip_samples(spi, 1U + (lh2_synth_rand(&rng) % 12U), &rng);
                break;
            case 1:  // jitter
                light.jitter = 0.3 + 0.7 * lh2_synth_uniform(&rng);
                lh2_synth_capture(&light, &rng, spi);
                break;
            case 2:  // dwell, either idle level
                light.dwell_us   = 2.0 + 14.0 * lh2_synth_uniform(&rng);
                light.idle_level = (uint8_t)(lh2_synth_rand(&rng) >> 31);
                lh2_synth_capture(&light, &rng, spi);
                break;
            case 3:  // mocap strobe
                lh2_synth_capture(&light, &rng, spi);
                lh2_synth_mocap(spi, 8U + 8U * (lh2_synth_rand(&rng) % 6U), lh2_synth_rand(&rng) % 48U);
                break;
            case 4:  // collide with another station
            {
                lh2_synth_light_t b = _random_light(&rng);
                uint8_t           sa[DB_LH2_CAPTURE_BYTES], sb[DB_LH2_CAPTURE_BYTES];
                lh2_synth_capture(&light, &rng, sa);
                lh2_synth_capture(&b, &rng, sb);
                lh2_synth_collide(sa, sb, 16.0 * lh2_synth_uniform(&rng), spi);
                break;
            }
            case 5:  // confuse: aimed flips toward another polynomial
            {
                uint8_t where[39];
                uint8_t other = (uint8_t)(lh2_synth_rand(&rng) >> 27);
                uint8_t k     = lh2_synth_confuse(&light, (uint8_t)(lh2_synth_rand(&rng) & 1U), other, where);
                k             = (k == 0) ? 0 : (uint8_t)(1U + lh2_synth_rand(&rng) % k);
                for (uint8_t j = 0; j < k; j++) {
                    light.flip[where[j] / 32] |= 1U << (where[j] % 32);
                }
                lh2_synth_capture(&light, &rng, spi);
                break;
            }
            case 6:  // constant tails and constant captures
                lh2_synth_capture(&light, &rng, spi);
                memset(spi + (lh2_synth_rand(&rng) % DB_LH2_CAPTURE_BYTES), (lh2_synth_rand(&rng) & 1U) ? 0xFF : 0x00, 1);
                if (i % 64 == 6) {
                    memset(spi, (i & 64) ? 0xFF : 0x00, DB_LH2_CAPTURE_BYTES);
                }
                break;
            default:  // the same pass twice, a few us apart
                lh2_synth_capture(&light, &rng, spi);
                t = _next_t(&rng, t);
                _feed_both("impaired", i, 4005, spi, t, _now(&rng, t));
                light.count += 3U + (lh2_synth_rand(&rng) % 300U);
                lh2_synth_capture(&light, &rng, spi);
                t += 20U + (lh2_synth_rand(&rng) % 60U);
                break;
        }
        if (i % 5000 == 4999) {
            _both_init();
        }
        t = _next_t(&rng, t);
        _feed_both("impaired", i, 4005, spi, t, _now(&rng, t));
    }
    CHECK(_differ == before, "golden, impaired: %u differ", _differ - before);
}

static void test_golden_random(void) {
    lh2_synth_rng_t rng    = { 4006 };
    uint32_t        n      = _long ? 1000000 : 100000;
    uint32_t        before = _differ;
    uint32_t        t      = 1000000;
    _both_init();
    for (uint32_t i = 0; i < n; i++) {
        uint8_t spi[DB_LH2_CAPTURE_BYTES];
        for (uint32_t b = 0; b < DB_LH2_CAPTURE_BYTES; b++) {
            spi[b] = (uint8_t)(lh2_synth_rand(&rng) >> 24);
        }
        t = _next_t(&rng, t);
        _feed_both("random", i, 4006, spi, t, _now(&rng, t));
    }
    CHECK(_differ == before, "golden, random: %u of %u captures differ", _differ - before, n);
}

/// A run through both, read every 100 ms like the swarmit glue, the pairs compared too
static void _golden_stream(const char *set, uint32_t seed, const lh2_synth_station_t *stations, uint8_t n_stations, double duration_us, uint64_t base_us, uint32_t *pairs) {
    lh2_synth_rng_t rng         = { seed };
    size_t          n           = lh2_synth_stream(stations, n_stations, duration_us, base_us, &rng, _events, 40000);
    double          next_read   = 100000;
    uint32_t        pair_differ = 0;
    _both_init();
    for (size_t i = 0; i <= n; i++) {
        double t = (i < n) ? _events[i].t_true_us : next_read;
        while (next_read <= t) {
            for (uint8_t station = 0; station < DB_LH2_STATIONS; station++) {
                uint32_t      counts[2];
                db_lh2_pair_t pair;
                bool          want = _ref_take_pair(station, counts);
                bool          got  = db_lh2_decoder_take_pair(&_dec, station, &pair);
                pair_differ += want != got || (got && (pair.count[0] != counts[0] || pair.count[1] != counts[1] || pair.station != station));
                *pairs += got;
            }
            next_read += 100000;
        }
        if (i == n) {
            break;
        }
        uint32_t now = _events[i].t_us + (lh2_synth_rand(&rng) % 400U);
        _feed_both(set, (uint32_t)i, seed, _events[i].spi, _events[i].t_us, now);
    }
    if (pair_differ) {
        _differ++;
        printf("  DIFFER %s seed %u: %u pair takes\n", set, seed, pair_differ);
    }
}

static void test_golden_streams(void) {
    uint32_t before = _differ, pairs = 0, start = _compared;
    uint32_t runs = _long ? 16 : 4;
    for (uint32_t r = 0; r < runs; r++) {
        lh2_synth_station_t sts[4] = {
            { .station = 1, .counts = { 30000, 70000 }, .t0_us = 1000, .dup_share = 0.3, .dup_delay_us = { 20, 60 }, .dup_sweeps = 1, .latency_us = 300 },
            { .station = 4, .counts = { 25000, 61000 }, .t0_us = 3100, .drop_share = 0.1, .silent_us = { 3000000, 5500000 }, .latency_us = 1500 },
            { .station = 9, .counts = { 35000, 81000 }, .t0_us = 7300, .dup_share = 0.1, .dup_delay_us = { 5, 120 }, .dup_sweeps = 3 },
            { .station = 15, .counts = { 2000, 104000 }, .t0_us = 11000, .latency_us = 3000, .drop_share = 0.3 },
        };
        uint8_t  n_stations = (uint8_t)(1U + r % 4U);
        uint64_t base       = (r % 2) ? 0x100000000ULL - 3000000ULL - r * 1000ULL : 1000000ULL;
        _golden_stream("stream", 5000U + r, sts, n_stations, 60000000.0 / n_stations, base, &pairs);
    }
    // two stations on one channel
    lh2_synth_station_t shared[2] = {
        { .station = 6, .counts = { 30000, 70000 }, .t0_us = 1000 },
        { .station = 6, .counts = { 41000, 90000 }, .t0_us = 5900 },
    };
    _golden_stream("shared channel", 5100, shared, 2, 8000000, 1000000, &pairs);
    CHECK(_differ == before, "golden, streams: %u of %u captures or pair takes differ, %u pairs taken", _differ - before, _compared - start, pairs);
}

#if !defined(LH2_MOCAP_FILTER)
/// Rows D6 and D7 with the motion-capture check off, the default: declared expectation changes
static void test_d6_d7_mocap_off(void) {
    lh2_synth_rng_t rng      = { 2606 };
    const double    dwells[] = { 2, 4, 6, 7, 7.75, 8, 9, 12, 16 };
    uint32_t        n = 2000, mocap = 0, wrong = 0;
    uint32_t        ok[9] = { 0 };
    for (uint32_t d = 0; d < sizeof(dwells) / sizeof(dwells[0]); d++) {
        for (uint32_t i = 0; i < n; i++) {
            lh2_synth_light_t light = _random_light(&rng);
            light.dwell_us          = dwells[d];
            light.idle_level        = (uint8_t)(lh2_synth_rand(&rng) >> 31);
            // The same sweep one revolution earlier, a few counts back: a weak
            // match is kept once its station has given that slot a count nearby
            lh2_synth_light_t before = light;
            before.count             = light.count > 3 ? light.count - 3 : 0;
            before.polynomial        = (uint8_t)((light.polynomial & ~1U) | (lh2_synth_rand(&rng) >> 31));
            db_lh2_capture_t prime   = { .t_us = 1000 };
            lh2_synth_capture(&before, &rng, prime.spi);
            uint32_t         t   = 1000 + db_lh2_period_us(light.polynomial >> 1);
            db_lh2_capture_t cap = { .t_us = t };
            lh2_synth_capture(&light, &rng, cap.spi);
            db_lh2_decoder_t dec;
            db_lh2_decoder_init(&dec);
            mocap += db_lh2_decoder_feed(&dec, &prime, 1000) == DB_LH2_DECODE_MOCAP;
            db_lh2_decode_result_t r = db_lh2_decoder_feed(&dec, &cap, t);
            mocap += r == DB_LH2_DECODE_MOCAP;
            uint8_t station = light.polynomial >> 1;
            bool    stored  = r == DB_LH2_DECODE_STORED;
            uint8_t sweep   = dec.slot[0][station].ready ? 0 : 1;
            int32_t e       = (int32_t)dec.slot[sweep][station].count - (int32_t)light.count;
            if (stored && dec.slot[sweep][station].ready && e >= -2 && e <= 2) {
                ok[d]++;
            } else if (stored) {
                wrong++;
            }
        }
    }
    MEASURED("D6 mocap check off, after a capture of the same sweep a revolution before: decoded right of %u at dwell 2, 4, 6, 7, 7.75, 8, 9, 12, 16 us: %u %u %u %u %u %u %u %u %u; %u wrong", n, ok[0], ok[1], ok[2], ok[3], ok[4], ok[5], ok[6], ok[7], ok[8], wrong);
    CHECK(mocap == 0, "D6 (changed): nothing is rejected as motion capture, %u were", mocap);
    // The shortest window needs 8 + 17 + 19 bits, about 7.3 us of light: dwells
    // the filter rejected up to 7.75 us now decode from about 7 us on.
    // A weak match also needs the capture before it decoded: at 7.75 us both did about 87 % of the time
    CHECK(ok[1] == 0 && ok[3] > 0 && ok[4] > n * 65 / 100 && ok[5] > n * 85 / 100 && ok[8] > n * 99 / 100, "D6 (changed): decoded from the shortest dwell that carries enough bits: %u at 4 us, %u at 7 us, %u at 7.75 us, %u at 8 us, %u at 16 us", ok[1], ok[3], ok[4], ok[5], ok[8]);

    // D7: a strobe is still never taken for a station's sweep
    uint32_t strobe_stored = 0;
    for (uint32_t period = 8; period <= 64; period += 8) {
        for (uint32_t phase = 0; phase < period; phase++) {
            db_lh2_capture_t cap = { .t_us = 1000 };
            memset(cap.spi, 0, sizeof(cap.spi));
            lh2_synth_mocap(cap.spi, period, phase);
            db_lh2_decoder_t dec;
            db_lh2_decoder_init(&dec);
            strobe_stored += db_lh2_decoder_feed(&dec, &cap, 1000) == DB_LH2_DECODE_STORED;
        }
    }
    CHECK(strobe_stored == 0, "D7: a strobe of any period from 0.5 to 4 MHz is never stored as a sweep, %u were", strobe_stored);
}
#endif

/// Four stations, both sweeps decoded once on clean captures, as at rest under them
static void _prime(db_lh2_decoder_t *dec, lh2_synth_rng_t *rng, uint8_t stations[4], uint32_t counts[4][2]) {
    db_lh2_decoder_init(dec);
    for (uint8_t i = 0; i < 4; i++) {
        bool fresh;
        do {
            stations[i] = (uint8_t)(lh2_synth_rand(rng) >> 28);
            fresh       = true;
            for (uint8_t j = 0; j < i; j++) {
                fresh = fresh && stations[j] != stations[i];
            }
        } while (!fresh);
        for (uint8_t sweep = 0; sweep < 2; sweep++) {
            counts[i][sweep]        = 20000U + sweep * 40000U + (uint32_t)(lh2_synth_uniform(rng) * 30000);
            lh2_synth_light_t light = _light(rng, (uint8_t)(stations[i] * 2U + (lh2_synth_rand(rng) >> 31)), counts[i][sweep]);
            db_lh2_capture_t  cap   = { .t_us = 1000U + i * 3000U + sweep * 7000U };
            lh2_synth_capture(&light, rng, cap.spi);
            db_lh2_decoder_feed(dec, &cap, cap.t_us);
        }
    }
}

/// Rows D3 and D4 with stations already heard: a weak match is no longer
/// enough to make another station, flipped or aimed
static void test_d3_d4_wrong_station(void) {
    lh2_synth_rng_t  rng = { 2303 };
    db_lh2_decoder_t primed, dec;
    uint8_t          stations[4];
    uint32_t         counts[4][2];
    uint32_t         trials = _long ? 240000 : 48000, wrong = 0, ref_wrong = 0, right = 0, ref_right = 0;
    for (uint32_t i = 0; i < trials; i++) {
        if (i % 1000 == 0) {
            _prime(&primed, &rng, stations, counts);
        }
        uint8_t           k     = (uint8_t)(i % 4);
        uint8_t           sweep = (uint8_t)((i / 4) % 2);
        lh2_synth_light_t light = _light(&rng, (uint8_t)(stations[k] * 2U + (lh2_synth_rand(&rng) >> 31)), counts[k][sweep] + (lh2_synth_rand(&rng) % 40));
        db_lh2_capture_t  cap   = { .t_us = 100000U + i };
        lh2_synth_capture(&light, &rng, cap.spi);
        lh2_synth_flip_samples(cap.spi, 1 + (uint32_t)(i / 4000) % 12, &rng);
        dec                      = primed;
        db_lh2_decode_result_t r = db_lh2_decoder_feed(&dec, &cap, cap.t_us);
        int8_t                 offset;
        uint8_t                p = db_lh2_polynomial(db_lh2_demodulate(cap.spi), &offset);
        if (r == DB_LH2_DECODE_STORED) {
            wrong += (p >> 1) != stations[k];
            right += (p >> 1) == stations[k];
        }
        // what the frozen copy's rule kept: any stored match, weak or not
        if (p != DB_LH2_POLYNOMIAL_NONE && r != DB_LH2_DECODE_NO_POLYNOMIAL) {
            ref_wrong += (p >> 1) != stations[k] && r != DB_LH2_DECODE_STORED && r == DB_LH2_DECODE_UNCONFIRMED;
            ref_right += (p >> 1) == stations[k];
        }
    }
    MEASURED("D3 flips 1-12 under four stations already heard: %u of %u flipped captures stored, %u to a wrong station (%u wrong ones refused as weak); %u matched their own station", right + wrong, trials, wrong, ref_wrong, ref_right);
    CHECK(wrong == 0, "D3 (changed): flips no longer make another station, %u did", wrong);

    // D4: flips aimed at a station that is also in view
    uint32_t aimed = 0, made_other = 0, weak_made = 0, weak_aimed = 0, min_flips = 99;
    for (uint32_t i = 0; i < 300; i++) {
        _prime(&primed, &rng, stations, counts);
        uint8_t           k = (uint8_t)(i % 4), target_k = (uint8_t)((k + 1 + i % 3) % 4);
        lh2_synth_light_t light = _light(&rng, (uint8_t)(stations[k] * 2U), counts[k][0]);
        uint8_t           where[39];
        uint8_t           target = (uint8_t)(stations[target_k] * 2U + (lh2_synth_rand(&rng) >> 31));
        uint8_t           n      = lh2_synth_confuse(&light, (uint8_t)(lh2_synth_rand(&rng) >> 31), target, where);
        for (uint8_t f = 1; f <= n; f++) {
            lh2_synth_light_t a = light;
            for (uint8_t j = 0; j < f; j++) {
                a.flip[where[j] / 32] |= 1U << (where[j] % 32);
            }
            db_lh2_capture_t cap = { .t_us = 200000U };
            lh2_synth_capture(&a, &rng, cap.spi);
            dec = primed;
            int8_t  offset;
            uint8_t compared;
            uint8_t p = db_lh2_polynomial_match(db_lh2_demodulate(cap.spi), &offset, &compared);
            aimed += p == target;
            weak_aimed += p == target && compared < DB_LH2_WEAK_MATCH_BITS;
            if (db_lh2_decoder_feed(&dec, &cap, cap.t_us) == DB_LH2_DECODE_STORED && p == target) {
                made_other++;
                weak_made += compared < DB_LH2_WEAK_MATCH_BITS;
                min_flips = (f < min_flips) ? f : min_flips;
            }
        }
    }
    MEASURED("D4 aimed flips at a station also in view: %u captures matched the aimed-at polynomial, %u of them on a weak match; %u stored, %u on a weak match, the fewest flips %u", aimed, weak_aimed, made_other, weak_made, min_flips);
    // The baseline's 3 aimed flips made another station on the shortest window. A weak match
    // now has to land within DB_LH2_CONFIRM_COUNTS of that station's last count, which an
    // aimed one does by chance; a strong one takes 6 flips or more, a forged sweep, not noise
    CHECK(weak_aimed > 500 && weak_made * 100 <= weak_aimed && min_flips >= 5, "D4 (changed): a few aimed flips rarely make another station: %u of %u weak matches stored, the fewest flips %u", weak_made, weak_aimed, min_flips);
}

/// Row S5, changed: a capture stamped exactly 0 holds its slot like any other
static void test_s5_stamp_zero(void) {
    lh2_synth_rng_t   rng = { 2505 };
    db_lh2_decoder_t  dec;
    db_lh2_capture_t  a = { .t_us = 0 }, b = { .t_us = 6000 };
    lh2_synth_light_t la = _light(&rng, 6, 30000), lb = _light(&rng, 6, 66000);
    lh2_synth_capture(&la, &rng, a.spi);
    lh2_synth_capture(&lb, &rng, b.spi);
    db_lh2_decoder_init(&dec);
    bool stored = db_lh2_decoder_feed(&dec, &a, 0) == DB_LH2_DECODE_STORED && db_lh2_decoder_feed(&dec, &b, 6000) == DB_LH2_DECODE_STORED;
    CHECK(stored && dec.slot[0][3].filled && dec.slot[0][3].t_us == 0 && dec.slot[1][3].filled && dec.slot[1][3].t_us == 6000 && db_lh2_decoder_pair_ready(&dec, 3),
          "S5 (changed): a sweep stamped 0 keeps slot 0, the station's other sweep takes slot 1 and the pair is ready");
}

static void test_counters(void) {
    db_lh2_decoder_t dec;
    db_lh2_decoder_init(&dec);
    uint8_t          zero[DB_LH2_CAPTURE_BYTES] = { 0 };
    db_lh2_capture_t cap                        = { .t_us = 5 };
    memcpy(cap.spi, zero, sizeof(zero));
    db_lh2_decoder_feed(&dec, &cap, 10);
    db_lh2_decoder_feed(&dec, &cap, 10);
#if defined(LH2_MOCAP_FILTER)
    CHECK(dec.results[DB_LH2_DECODE_MOCAP] == 2 && dec.results[DB_LH2_DECODE_STORED] == 0, "two all-zero captures count as motion capture: %u", dec.results[DB_LH2_DECODE_MOCAP]);
#else
    CHECK(dec.results[DB_LH2_DECODE_NO_POLYNOMIAL] == 2 && dec.results[DB_LH2_DECODE_STORED] == 0, "two all-zero captures count as no polynomial: %u", dec.results[DB_LH2_DECODE_NO_POLYNOMIAL]);
#endif
    CHECK(!db_lh2_decoder_pair_ready(&dec, 16), "no pair for a station out of range");
}

//=========================== queue ============================================

/// The 228ce9a SPI ring, its masking left out
typedef struct {
    uint8_t  buffer[10][64];
    uint32_t timestamps[10];
    uint8_t  write_index;
    uint8_t  read_index;
    uint8_t  count;
} ring_t;

static void _ring_add(ring_t *cb, const uint8_t *data, uint32_t timestamp) {
    memcpy(cb->buffer[cb->write_index], data, 64);
    cb->timestamps[cb->write_index] = timestamp;
    cb->write_index                 = (cb->write_index + 1) % 10;
    if (cb->count < 10) {
        cb->count++;
    } else {
        cb->read_index = (cb->read_index + 1) % 10;
    }
}

static bool _ring_get(ring_t *cb, uint8_t *data, uint32_t *timestamp) {
    if (cb->count == 0) {
        return false;
    }
    memcpy(data, cb->buffer[cb->read_index], 64);
    *timestamp     = cb->timestamps[cb->read_index];
    cb->read_index = (cb->read_index + 1) % 10;
    cb->count--;
    return true;
}

static void test_queue(void) {
    db_lh2_queue_t  q      = { 0 };
    ring_t          ring   = { 0 };
    lh2_synth_rng_t rng    = { 4007 };
    uint32_t        differ = 0, overwritten = 0, ops = 200000;
    for (uint32_t i = 0; i < ops; i++) {
        if (lh2_synth_rand(&rng) % 100 < 55) {
            uint8_t spi[DB_LH2_CAPTURE_BYTES];
            for (uint32_t b = 0; b < DB_LH2_CAPTURE_BYTES; b++) {
                spi[b] = (uint8_t)(lh2_synth_rand(&rng) >> 24);
            }
            overwritten += ring.count == 10;
            _ring_add(&ring, spi, i);
            db_lh2_queue_push(&q, spi, i);
        } else {
            uint8_t          data[64];
            uint32_t         ts = 0;
            db_lh2_capture_t cap;
            bool             want = _ring_get(&ring, data, &ts);
            bool             got  = db_lh2_queue_pop(&q, &cap);
            differ += want != got || (got && (cap.t_us != ts || memcmp(cap.spi, data, 64) != 0));
        }
        differ += q.count != ring.count;
    }
    CHECK(differ == 0, "queue identical to the 228ce9a ring over %u random pushes and pops: %u differ", ops, differ);
    CHECK(q.dropped == overwritten && overwritten > 0, "queue counts every overwritten capture: %u counted, %u overwritten", q.dropped, overwritten);
    db_lh2_queue_clear(&q);
    db_lh2_capture_t cap;
    CHECK(!db_lh2_queue_pop(&q, &cap) && q.dropped == overwritten, "clear empties the queue and keeps the drop counter");
}

int main(int argc, char **argv) {
    _long = argc > 1 && strcmp(argv[1], "--long") == 0;
    test_tables();
    test_init_and_reset();
    test_low_layer();
    test_lfsr_index_every_position();
    test_golden_clean();
    test_golden_impaired();
    test_golden_random();
    test_golden_streams();
#if !defined(LH2_MOCAP_FILTER)
    test_d6_d7_mocap_off();
#endif
    test_d3_d4_wrong_station();
    test_s5_stamp_zero();
    test_counters();
    test_queue();
#if defined(LH2_MOCAP_FILTER)
    MEASURED("built with LH2_MOCAP_FILTER: the frozen copy's behaviour, every capture compared");
#else
    MEASURED("declared divergence, the motion-capture check off: %u captures the frozen copy rejects as mocap, %u of them rejected here too", _declared_mocap, _declared_mocap_kept);
    CHECK(_declared_mocap_kept == 0, "declared divergence: a capture the frozen copy takes for motion capture is decoded like any other, %u were not", _declared_mocap_kept);
#endif
    MEASURED("declared divergence, weak matches: %u not confirmed, %u of which the frozen copy stored", _declared_weak, _declared_weak_stored);
    CHECK(_declared_wrong == 0, "declared divergence: every capture outside the declared classes compared, and every declared outcome held; %u did not", _declared_wrong);
    MEASURED("declared divergence, explicit slot occupancy: %u captures stamped 0, or fed while a slot held one", _declared_s5);
    MEASURED("golden comparison: %u captures, %u differ; reference results stored %u, mocap %u, no polynomial %u, zero bits %u, no index %u, past revolution %u, equal counts %u",
             _compared, _differ, _ref_results[0], _ref_results[1], _ref_results[2], _ref_results[3], _ref_results[4], _ref_results[5], _ref_results[6]);
#if defined(LH2_MOCAP_FILTER)
    printf("lh2_decoder (mocap filter)%s: %d passed, %d failed\n", _long ? " (long)" : "", _passed, _failed);
#else
    printf("lh2_decoder%s: %d passed, %d failed\n", _long ? " (long)" : "", _passed, _failed);
#endif
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
