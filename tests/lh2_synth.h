#ifndef __LH2_SYNTH_H
#define __LH2_SYNTH_H

/**
 * @file
 * @brief  Synthetic LH2 captures for the host tests
 *
 * A station's rotor restarts its LFSR from state 1 at angle 0 of every
 * revolution and emits one bit per count (8 ticks of 48 MHz, so 6 Mbit/s);
 * which polynomial of its pair it runs carries that revolution's data bit.
 * The bits are biphase-mark coded: a transition at every bit boundary, and a
 * second one mid-bit for a 1. A capture is the TS4231 data line sampled at
 * 32 MHz, MSB first, for 64 bytes from the envelope's falling edge, plus the
 * time the SPIM handler stamped it with.
 *
 * Captures are built from the published polynomials alone, never from the
 * decoder under test. Random knobs draw from a caller-owned 32-bit LCG.
 *
 * @copyright Inria, 2026
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//=========================== defines ==========================================

#define LH2_SYNTH_CAPTURE_BYTES   (64U)
#define LH2_SYNTH_SAMPLES         (512U)
#define LH2_SYNTH_SAMPLES_PER_BIT (32.0 / 6.0)
#define LH2_SYNTH_SAMPLES_PER_US  (32.0)
#define LH2_SYNTH_COUNTS_PER_US   (6.0)
#define LH2_SYNTH_CAPTURE_US      (16.0)     ///< 512 samples at 32 MHz
#define LH2_SYNTH_BITS            (100U)     ///< stream bits a capture can reach, with margin
#define LH2_SYNTH_LFSR_PERIOD     (131071U)  ///< 2^17 - 1
#define LH2_SYNTH_POLYNOMIALS     (32U)
#define LH2_SYNTH_STATIONS        (16U)
#define LH2_SYNTH_MAX_STATIONS    (4U)  ///< in one stream

typedef struct {
    uint32_t state;
} lh2_synth_rng_t;

/// The light of one pass, as the knobs that act inside one capture set it
typedef struct {
    uint8_t  polynomial;  ///< 0..31; the station is polynomial >> 1
    uint32_t count;       ///< n0: the LFSR index of the bit sample 0 falls in
    double   phase;       ///< phase: where sample 0 falls inside that bit, samples, [0, 32/6)
    uint8_t  level;       ///< the data line level before the first edge
    double   jitter;      ///< jitter: sigma of Gaussian jitter on every edge, samples
    double   dwell_us;    ///< dwell: light for this long, then idle_level; 0 for light throughout
    uint8_t  idle_level;  ///< the data line once the light is gone
    uint32_t flip[4];     ///< confuse: stream bits inverted before coding, bit i is the bit at n0 + i
} lh2_synth_light_t;

/// One station of a stream, with the knobs that act across captures
typedef struct {
    uint8_t  station;          ///< lh_index 0..15
    uint32_t counts[2];        ///< the count of each sweep, every revolution
    double   t0_us;            ///< the start of revolution 0, from the start of the run
    double   dup_share;        ///< dup-late: the share of passes with a second capture
    double   dup_delay_us[2];  ///< dup-late: its delay after the first, uniform in [min, max]
    uint8_t  dup_sweeps;       ///< dup-late: bit s set when sweep s can get one
    double   drop_share;       ///< drop: the share of passes with no capture
    double   silent_us[2];     ///< drop: no capture from silent_us[0] to silent_us[1]
    uint8_t  first_drop;       ///< drop: bit s set to drop sweep s of revolution 0
    double   dwell_us;         ///< truncate: every capture's dwell; 0 for full captures
    double   latency_us;       ///< latency: added to each stamp, uniform in [0, latency_us]
} lh2_synth_station_t;

/// One capture of a stream, with the truth it was made from
typedef struct {
    uint8_t  spi[LH2_SYNTH_CAPTURE_BYTES];
    uint32_t t_us;        ///< the handler's stamp: the end of the capture plus latency, on the 32-bit timer
    double   t_true_us;   ///< when the light arrived, from the start of the run
    uint8_t  station;     ///< lh_index
    uint8_t  sweep;       ///< 0: the station's first sweep of a revolution, 1: its second
    uint8_t  polynomial;  ///< 0..31
    uint32_t count;       ///< n0
    uint32_t revolution;
    bool     duplicate;  ///< the second capture of a pass
} lh2_synth_event_t;

//=========================== prototypes =======================================

/// The next 32-bit LCG draw
uint32_t lh2_synth_rand(lh2_synth_rng_t *rng);

/// Uniform in [0, 1)
double lh2_synth_uniform(lh2_synth_rng_t *rng);

/// Standard normal
double lh2_synth_gauss(lh2_synth_rng_t *rng);

/// The published LFSR polynomial of index 0..31; 0 out of range
uint32_t lh2_synth_polynomial(uint8_t index);

/// The 17 stream bits at index n of a polynomial's sequence from state 1, oldest the MSB
uint32_t lh2_synth_lfsr_state(uint8_t polynomial, uint32_t n);

/// Stream bits n .. n + nbits - 1, one per byte
void lh2_synth_lfsr_bits(uint8_t polynomial, uint32_t n, uint8_t *bits, uint32_t nbits);

/// LFSR index of a 17-bit state, by brute force; LH2_SYNTH_LFSR_PERIOD when none
uint32_t lh2_synth_lfsr_index(uint8_t polynomial, uint32_t state);

/**
 * @brief   Biphase-mark code a bit stream and sample it as the SPIM does
 *
 * @param[in]   bits    one bit per byte, bit 0 the one sample 0 falls in; at least LH2_SYNTH_BITS
 * @param[in]   phase   where sample 0 falls inside bit 0, samples
 * @param[in]   level   the line level before the first edge
 * @param[in]   jitter  sigma of Gaussian jitter on every edge, samples; 0 draws nothing
 */
void lh2_synth_code(const uint8_t *bits, double phase, uint8_t level, double jitter, lh2_synth_rng_t *rng, uint8_t spi[LH2_SYNTH_CAPTURE_BYTES]);

/// The capture of one pass; rng is used only for jitter
void lh2_synth_capture(const lh2_synth_light_t *light, lh2_synth_rng_t *rng, uint8_t spi[LH2_SYNTH_CAPTURE_BYTES]);

/// dwell: every sample from dwell_us on set to the idle level
void lh2_synth_dwell(uint8_t spi[LH2_SYNTH_CAPTURE_BYTES], double dwell_us, uint8_t idle_level);

/// flips: k distinct random samples inverted
void lh2_synth_flip_samples(uint8_t spi[LH2_SYNTH_CAPTURE_BYTES], uint32_t k, lh2_synth_rng_t *rng);

/// mocap: a square wave, period_samples per cycle, its first half high from phase_samples
void lh2_synth_mocap(uint8_t spi[LH2_SYNTH_CAPTURE_BYTES], uint32_t period_samples, uint32_t phase_samples);

/// collide: first's samples up to offset_us, second's from there
void lh2_synth_collide(const uint8_t first[LH2_SYNTH_CAPTURE_BYTES], const uint8_t second[LH2_SYNTH_CAPTURE_BYTES], double offset_us, uint8_t out[LH2_SYNTH_CAPTURE_BYTES]);

/**
 * @brief   confuse: the stream bits that make a pass read as another polynomial
 *
 * With the 64 demodulated bits starting at stream bit `shift` of the pass,
 * the 17 at word offset 8 seed `other`, and every later word bit that its
 * LFSR run from that seed disagrees with is an aimed flip.
 *
 * @param[in]   light   the pass
 * @param[in]   shift   the stream bit the demodulated word starts at, 0 or 1
 * @param[in]   other   the polynomial to aim at
 * @param[out]  where   stream bits to invert, earliest first, at most 39
 *
 * @return  the number of aimed flips
 */
uint8_t lh2_synth_confuse(const lh2_synth_light_t *light, uint8_t shift, uint8_t other, uint8_t where[39]);

/// Rotor period of a station, us
double lh2_synth_period_us(uint8_t station);

/**
 * @brief   The captures of a run, in arrival order
 *
 * @param[in]   stations    1 to LH2_SYNTH_MAX_STATIONS
 * @param[in]   duration_us the run
 * @param[in]   base_us     wrap: the 32-bit timer at the start of the run, so 2^32 falls where wanted
 * @param[out]  out         at most max events
 *
 * @return  the number of events; max when the run did not fit
 */
size_t lh2_synth_stream(const lh2_synth_station_t *stations, uint8_t n_stations, double duration_us, uint64_t base_us, lh2_synth_rng_t *rng, lh2_synth_event_t *out, size_t max);

#endif
