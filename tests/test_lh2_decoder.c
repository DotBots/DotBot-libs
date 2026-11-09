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
            if (s->count != _ref.locations[sweep][station].lfsr_counts || (uint32_t)s->polynomial != _ref.locations[sweep][station].selected_polynomial || s->t_us != _ref.timestamps[sweep][station] || s->ready != ready_ref) {
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

/// One capture into both; false, and the case printed, on any difference
static bool _feed_both(const char *set, uint32_t index, uint32_t seed, const uint8_t spi[DB_LH2_CAPTURE_BYTES], uint32_t t_us, uint32_t now_us) {
    db_lh2_capture_t cap;
    memcpy(cap.spi, spi, DB_LH2_CAPTURE_BYTES);
    cap.t_us                       = t_us;
    db_lh2_decode_result_t want    = _from_ref(lh2_ref_feed(&_ref, spi, t_us, now_us));
    uint32_t               counted = _dec.results[want < DB_LH2_DECODE_RESULTS ? want : 0];
    db_lh2_decode_result_t got     = db_lh2_decoder_feed(&_dec, &cap, now_us);
    char                   buf[200];
    const char            *differ = (got != want) ? "the result" : _state_differs(buf, sizeof(buf));
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

static void test_counters(void) {
    db_lh2_decoder_t dec;
    db_lh2_decoder_init(&dec);
    uint8_t          zero[DB_LH2_CAPTURE_BYTES] = { 0 };
    db_lh2_capture_t cap                        = { .t_us = 5 };
    memcpy(cap.spi, zero, sizeof(zero));
    db_lh2_decoder_feed(&dec, &cap, 10);
    db_lh2_decoder_feed(&dec, &cap, 10);
    CHECK(dec.results[DB_LH2_DECODE_MOCAP] == 2 && dec.results[DB_LH2_DECODE_STORED] == 0, "two all-zero captures count as motion capture: %u", dec.results[DB_LH2_DECODE_MOCAP]);
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
    test_counters();
    test_queue();
    MEASURED("golden comparison: %u captures, %u differ; reference results stored %u, mocap %u, no polynomial %u, zero bits %u, no index %u, past revolution %u, equal counts %u",
             _compared, _differ, _ref_results[0], _ref_results[1], _ref_results[2], _ref_results[3], _ref_results[4], _ref_results[5], _ref_results[6]);
    printf("lh2_decoder%s: %d passed, %d failed\n", _long ? " (long)" : "", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
