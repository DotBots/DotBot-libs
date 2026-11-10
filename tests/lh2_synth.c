/**
 * @file
 * @brief  Synthetic LH2 captures for the host tests
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "lh2_geometry.h"
#include "lh2_synth.h"

//=========================== variables ========================================

/// The Lighthouse 2 LFSR polynomials, two per channel
static const uint32_t _polynomials[LH2_SYNTH_POLYNOMIALS] = {
    0x0001D258, 0x00017E04, 0x0001FF6B, 0x00013F67, 0x0001B9EE, 0x000198D1, 0x000178C7, 0x00018A55,
    0x00015777, 0x0001D911, 0x00015769, 0x0001991F, 0x00012BD0, 0x0001CF73, 0x0001365D, 0x000197F5,
    0x000194A0, 0x0001B279, 0x00013A34, 0x0001AE41, 0x000180D4, 0x00017891, 0x00012E64, 0x00017C72,
    0x00019C6D, 0x00013F32, 0x0001AE14, 0x00014E76, 0x00013C97, 0x000130CB, 0x00013750, 0x0001CB8D
};

#define STRIDE  (1024U)  ///< LFSR states kept every STRIDE indices
#define STRIDES ((LH2_SYNTH_LFSR_PERIOD / STRIDE) + 1U)

static uint32_t _strided[LH2_SYNTH_POLYNOMIALS][STRIDES];
static bool     _strided_ready[LH2_SYNTH_POLYNOMIALS];

//=========================== private ==========================================

static uint32_t _step(uint32_t state, uint32_t polynomial) {
    return ((state << 1) | (__builtin_popcount(state & polynomial) & 1U)) & 0x1FFFFU;
}

static void _fill_strided(uint8_t p) {
    uint32_t state = 1;
    for (uint32_t n = 0; n < LH2_SYNTH_LFSR_PERIOD; n++) {
        if (n % STRIDE == 0) {
            _strided[p][n / STRIDE] = state;
        }
        state = _step(state, _polynomials[p]);
    }
    _strided_ready[p] = true;
}

static int _by_time(const void *a, const void *b) {
    double ta = ((const lh2_synth_event_t *)a)->t_true_us;
    double tb = ((const lh2_synth_event_t *)b)->t_true_us;
    return (ta > tb) - (ta < tb);
}

static void _set_sample(uint8_t spi[LH2_SYNTH_CAPTURE_BYTES], uint32_t k, uint8_t level) {
    uint8_t mask = (uint8_t)(0x80U >> (k % 8U));
    if (level) {
        spi[k / 8U] |= mask;
    } else {
        spi[k / 8U] &= (uint8_t)~mask;
    }
}

//=========================== public ===========================================

uint32_t lh2_synth_rand(lh2_synth_rng_t *rng) {
    rng->state = rng->state * 1664525U + 1013904223U;
    return rng->state;
}

double lh2_synth_uniform(lh2_synth_rng_t *rng) {
    return (double)(lh2_synth_rand(rng) >> 8) / 16777216.0;
}

double lh2_synth_gauss(lh2_synth_rng_t *rng) {
    double u1 = lh2_synth_uniform(rng);
    double u2 = lh2_synth_uniform(rng);
    return sqrt(-2.0 * log(1.0 - u1)) * cos(2.0 * M_PI * u2);
}

uint32_t lh2_synth_polynomial(uint8_t index) {
    return (index < LH2_SYNTH_POLYNOMIALS) ? _polynomials[index] : 0;
}

uint32_t lh2_synth_lfsr_state(uint8_t polynomial, uint32_t n) {
    polynomial %= LH2_SYNTH_POLYNOMIALS;
    n %= LH2_SYNTH_LFSR_PERIOD;
    if (!_strided_ready[polynomial]) {
        _fill_strided(polynomial);
    }
    uint32_t state = _strided[polynomial][n / STRIDE];
    for (uint32_t i = 0; i < n % STRIDE; i++) {
        state = _step(state, _polynomials[polynomial]);
    }
    return state;
}

void lh2_synth_lfsr_bits(uint8_t polynomial, uint32_t n, uint8_t *bits, uint32_t nbits) {
    uint32_t state = lh2_synth_lfsr_state(polynomial, n);
    for (uint32_t i = 0; i < nbits; i++) {
        bits[i] = (uint8_t)((state >> 16) & 1U);
        state   = _step(state, _polynomials[polynomial % LH2_SYNTH_POLYNOMIALS]);
    }
}

uint32_t lh2_synth_lfsr_index(uint8_t polynomial, uint32_t state) {
    uint32_t s = 1;
    for (uint32_t n = 0; n < LH2_SYNTH_LFSR_PERIOD; n++) {
        if (s == state) {
            return n;
        }
        s = _step(s, _polynomials[polynomial % LH2_SYNTH_POLYNOMIALS]);
    }
    return LH2_SYNTH_LFSR_PERIOD;
}

void lh2_synth_code(const uint8_t *bits, double phase, uint8_t level, double jitter, lh2_synth_rng_t *rng, uint8_t spi[LH2_SYNTH_CAPTURE_BYTES]) {
    double   edges[2 * LH2_SYNTH_BITS];
    uint32_t n = 0;
    for (uint32_t i = 0; i < LH2_SYNTH_BITS; i++) {
        edges[n++] = i * LH2_SYNTH_SAMPLES_PER_BIT;
        if (bits[i]) {
            edges[n++] = (i + 0.5) * LH2_SYNTH_SAMPLES_PER_BIT;
        }
    }
    if (jitter > 0) {
        for (uint32_t i = 0; i < n; i++) {
            edges[i] += jitter * lh2_synth_gauss(rng);
        }
        // Nearly sorted: insertion sort
        for (uint32_t i = 1; i < n; i++) {
            double   e = edges[i];
            uint32_t j = i;
            while (j > 0 && edges[j - 1] > e) {
                edges[j] = edges[j - 1];
                j--;
            }
            edges[j] = e;
        }
    }
    memset(spi, 0, LH2_SYNTH_CAPTURE_BYTES);
    uint32_t next = 0;
    for (uint32_t k = 0; k < LH2_SYNTH_SAMPLES; k++) {
        double t = k + phase;
        while (next < n && edges[next] <= t) {
            level ^= 1U;
            next++;
        }
        _set_sample(spi, k, level);
    }
}

void lh2_synth_capture(const lh2_synth_light_t *light, lh2_synth_rng_t *rng, uint8_t spi[LH2_SYNTH_CAPTURE_BYTES]) {
    uint8_t bits[LH2_SYNTH_BITS];
    lh2_synth_lfsr_bits(light->polynomial, light->count, bits, LH2_SYNTH_BITS);
    for (uint32_t i = 0; i < LH2_SYNTH_BITS && i < 128U; i++) {
        bits[i] ^= (uint8_t)((light->flip[i / 32U] >> (i % 32U)) & 1U);
    }
    lh2_synth_code(bits, light->phase, light->level & 1U, light->jitter, rng, spi);
    if (light->dwell_us > 0) {
        lh2_synth_dwell(spi, light->dwell_us, light->idle_level);
    }
}

void lh2_synth_dwell(uint8_t spi[LH2_SYNTH_CAPTURE_BYTES], double dwell_us, uint8_t idle_level) {
    double first = ceil(dwell_us * LH2_SYNTH_SAMPLES_PER_US);
    for (uint32_t k = (first < 0) ? 0 : (uint32_t)first; k < LH2_SYNTH_SAMPLES; k++) {
        _set_sample(spi, k, idle_level & 1U);
    }
}

void lh2_synth_flip_samples(uint8_t spi[LH2_SYNTH_CAPTURE_BYTES], uint32_t k, lh2_synth_rng_t *rng) {
    uint8_t flipped[LH2_SYNTH_SAMPLES] = { 0 };
    k                                  = (k > LH2_SYNTH_SAMPLES) ? LH2_SYNTH_SAMPLES : k;
    for (uint32_t done = 0; done < k;) {
        uint32_t s = lh2_synth_rand(rng) >> 23;  // 0..511
        if (flipped[s]) {
            continue;
        }
        flipped[s] = 1;
        spi[s / 8U] ^= (uint8_t)(0x80U >> (s % 8U));
        done++;
    }
}

void lh2_synth_mocap(uint8_t spi[LH2_SYNTH_CAPTURE_BYTES], uint32_t period_samples, uint32_t phase_samples) {
    for (uint32_t k = 0; k < LH2_SYNTH_SAMPLES; k++) {
        _set_sample(spi, k, ((k + phase_samples) % period_samples) < period_samples / 2U);
    }
}

void lh2_synth_collide(const uint8_t first[LH2_SYNTH_CAPTURE_BYTES], const uint8_t second[LH2_SYNTH_CAPTURE_BYTES], double offset_us, uint8_t out[LH2_SYNTH_CAPTURE_BYTES]) {
    double from = ceil(offset_us * LH2_SYNTH_SAMPLES_PER_US);
    for (uint32_t k = 0; k < LH2_SYNTH_SAMPLES; k++) {
        const uint8_t *src = (k < from) ? first : second;
        _set_sample(out, k, (src[k / 8U] >> (7U - k % 8U)) & 1U);
    }
}

uint8_t lh2_synth_confuse(const lh2_synth_light_t *light, uint8_t shift, uint8_t other, uint8_t where[39]) {
    uint8_t bits[LH2_SYNTH_BITS];
    lh2_synth_lfsr_bits(light->polynomial, light->count, bits, LH2_SYNTH_BITS);
    uint32_t seed = 0;
    for (uint32_t j = 8; j < 25; j++) {
        seed = (seed << 1) | bits[shift + j];
    }
    uint8_t n = 0;
    for (uint32_t j = 25; j < 64; j++) {
        uint32_t next = __builtin_popcount(seed & lh2_synth_polynomial(other)) & 1U;
        seed          = ((seed << 1) | next) & 0x1FFFFU;
        if (next != bits[shift + j]) {
            where[n++] = (uint8_t)(shift + j);
        }
    }
    return n;
}

double lh2_synth_period_us(uint8_t station) {
    return db_lh2_period(station) / 48.0;
}

size_t lh2_synth_stream(const lh2_synth_station_t *stations, uint8_t n_stations, double duration_us, uint64_t base_us, lh2_synth_rng_t *rng, lh2_synth_event_t *out, size_t max) {
    size_t n = 0;
    for (uint8_t i = 0; i < n_stations && i < LH2_SYNTH_MAX_STATIONS; i++) {
        const lh2_synth_station_t *st     = &stations[i];
        double                     period = lh2_synth_period_us(st->station);
        for (uint32_t rev = 0;; rev++) {
            double t_rev = st->t0_us + rev * period;
            if (t_rev > duration_us) {
                break;
            }
            uint8_t polynomial = (uint8_t)(2U * st->station + (lh2_synth_rand(rng) >> 31));
            for (uint8_t sweep = 0; sweep < 2; sweep++) {
                double t = t_rev + st->counts[sweep] / LH2_SYNTH_COUNTS_PER_US;
                if (t > duration_us) {
                    continue;
                }
                if (rev == 0 && ((st->first_drop >> sweep) & 1U)) {
                    continue;
                }
                if (t >= st->silent_us[0] && t < st->silent_us[1]) {
                    continue;
                }
                if (st->drop_share > 0 && lh2_synth_uniform(rng) < st->drop_share) {
                    continue;
                }
                bool   dup   = ((st->dup_sweeps >> sweep) & 1U) && st->dup_share > 0 && lh2_synth_uniform(rng) < st->dup_share;
                double delay = st->dup_delay_us[0] + (st->dup_delay_us[1] - st->dup_delay_us[0]) * lh2_synth_uniform(rng);
                for (uint8_t copy = 0; copy < (dup ? 2 : 1); copy++) {
                    if (n == max) {
                        return max;
                    }
                    lh2_synth_event_t *ev    = &out[n++];
                    double             t_cap = t + (copy ? delay : 0);
                    lh2_synth_light_t  light = {
                         .polynomial = polynomial,
                         .count      = st->counts[sweep] + (copy ? (uint32_t)lround(delay * LH2_SYNTH_COUNTS_PER_US) : 0),
                         .phase      = lh2_synth_uniform(rng) * LH2_SYNTH_SAMPLES_PER_BIT,
                         .level      = (uint8_t)(lh2_synth_rand(rng) >> 31),
                         .dwell_us   = st->dwell_us,
                         .idle_level = (uint8_t)(lh2_synth_rand(rng) >> 31),
                    };
                    lh2_synth_capture(&light, rng, ev->spi);
                    double latency = st->latency_us * lh2_synth_uniform(rng);
                    ev->t_us       = (uint32_t)(base_us + (uint64_t)floor(t_cap + LH2_SYNTH_CAPTURE_US + latency));
                    ev->t_true_us  = t_cap;
                    ev->station    = st->station;
                    ev->sweep      = sweep;
                    ev->polynomial = polynomial;
                    ev->count      = light.count;
                    ev->revolution = rev;
                    ev->duplicate  = copy == 1;
                }
            }
        }
    }
    qsort(out, n, sizeof(out[0]), _by_time);
    return n;
}
