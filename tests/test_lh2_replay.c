/**
 * @file
 * @brief  Host test replaying recorded LH2 captures (row E7)
 *
 * Every fixture in tests/lh2_fixtures.h, written by dist/scripts/lh2/lh2_capture.py
 * from a recording, goes capture by capture through both the frozen 228ce9a
 * decode path and drv/lh2_decoder, each time the pairs taken every 100 ms of
 * capture time as the swarmit glue does; after each capture the result, the
 * slots and the dynamic checkpoints must be identical. Each fixture's outcome
 * is printed: what the decoder did with its captures, and its pairs.
 *
 * @copyright Inria, 2026
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lh2_decoder.h"
#include "lh2_fixtures.h"
#include "lh2_reference.h"

static int _failed = 0;
static int _passed = 0;

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

static lh2_ref_t        _ref;
static db_lh2_decoder_t _dec;

static bool _same_state(void) {
    for (uint8_t sweep = 0; sweep < DB_LH2_SWEEPS; sweep++) {
        for (uint8_t station = 0; station < DB_LH2_STATIONS; station++) {
            const db_lh2_slot_t *s     = &_dec.slot[sweep][station];
            bool                 ready = _ref.data_ready[sweep][station] == LH2_REF_PROCESSED_DATA_AVAILABLE;
            if (s->count != _ref.locations[sweep][station].lfsr_counts || (uint32_t)s->polynomial != _ref.locations[sweep][station].selected_polynomial || s->t_us != _ref.timestamps[sweep][station] || s->ready != ready) {
                return false;
            }
        }
    }
    const lh2_ref_checkpoint_t *cp = lh2_ref_checkpoints();
    return memcmp(_dec.checkpoint_bits, cp->bits, sizeof(cp->bits)) == 0 && memcmp(_dec.checkpoint_count, cp->count, sizeof(cp->count)) == 0;
}

/// The decoder's slots and checkpoints set to the frozen copy's
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

static void test_fixture(const lh2_fixture_t *f) {
    lh2_ref_init(&_ref);
    db_lh2_decoder_init(&_dec);
    uint32_t differ = 0, pairs = 0, pair_differ = 0, unconfirmed = 0;
    uint32_t next_read = f->count ? f->captures[0].t_us + 100000U : 0;
    for (size_t i = 0; i < f->count; i++) {
        db_lh2_capture_t cap;
        memcpy(cap.spi, f->captures[i].spi, DB_LH2_CAPTURE_BYTES);
        cap.t_us              = f->captures[i].t_us;
        lh2_ref_result_t want = lh2_ref_feed(&_ref, cap.spi, cap.t_us, cap.t_us);
#if !defined(LH2_MOCAP_FILTER)
        if (want == LH2_REF_MOCAP) {
            // declared: decoded like any other capture; the frozen copy's state did not move
            db_lh2_decoder_t before = _dec;
            differ += db_lh2_decoder_feed(&_dec, &cap, cap.t_us) == DB_LH2_DECODE_MOCAP;
            _dec = before;
            continue;
        }
#endif
        db_lh2_decode_result_t got = db_lh2_decoder_feed(&_dec, &cap, cap.t_us);
        if (got == DB_LH2_DECODE_UNCONFIRMED) {
            // declared: a weak match away from its station's last count is not kept
            unconfirmed++;
            _resync();
        } else if ((int)got != (int)want || !_same_state()) {
            differ++;
        }
        if ((int32_t)(cap.t_us - next_read) >= 0) {
            next_read += 100000U;
            for (uint8_t station = 0; station < DB_LH2_STATIONS; station++) {
                bool          ref_ready = _ref.data_ready[0][station] == LH2_REF_PROCESSED_DATA_AVAILABLE && _ref.data_ready[1][station] == LH2_REF_PROCESSED_DATA_AVAILABLE;
                db_lh2_pair_t pair;
                bool          took = db_lh2_decoder_take_pair(&_dec, station, &pair);
                if (ref_ready) {
                    _ref.data_ready[0][station] = LH2_REF_NO_NEW_DATA;
                    _ref.data_ready[1][station] = LH2_REF_NO_NEW_DATA;
                }
                pairs += took;
                pair_differ += took != ref_ready || (took && (pair.count[0] != _ref.locations[0][station].lfsr_counts || pair.count[1] != _ref.locations[1][station].lfsr_counts));
            }
        }
    }
    CHECK(differ == 0 && pair_differ == 0, "E7 %s: the decoder and the frozen copy agree on every capture: %u of %zu differ, %u of %u pairs", f->name, differ, f->count, pair_differ, pairs);
    printf("  lh2_replay: %s (%s): %zu captures, stored %u, mocap %u, no polynomial %u, zero bits %u, no index %u, past revolution %u, equal counts %u, unconfirmed %u; %u pairs\n",
           f->name, f->source, f->count, _dec.results[0], _dec.results[1], _dec.results[2], _dec.results[3], _dec.results[4], _dec.results[5], _dec.results[6], unconfirmed, pairs);
}

static void test_record_layout(void) {
    // The record dist/scripts/lh2/lh2_capture.py checks against, GOLDEN_RECORD
    static const uint8_t golden[DB_LH2_RECORD_BYTES] = {
        0x4c, 0x32, 0xef, 0xcd, 0xab, 0x89, 0x78, 0x56, 0x34, 0x12, 0x03, 0x00, 0x07, 0x00,
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
        0x6c, 0x0c
    };
    db_lh2_capture_t cap;
    for (uint8_t i = 0; i < DB_LH2_CAPTURE_BYTES; i++) {
        cap.spi[i] = i;
    }
    cap.t_us = 0x12345678U;
    uint8_t rec[DB_LH2_RECORD_BYTES];
    db_lh2_capture_record(&cap, 0x89ABCDEFU, 3, 7, rec);
    CHECK(memcmp(rec, golden, sizeof(rec)) == 0, "record: db_lh2_capture_record() writes the layout the fixture script reads");
    db_lh2_capture_record(&cap, 0, 70000, 1, rec);
    CHECK(rec[10] == 0xFF && rec[11] == 0xFF && rec[12] == 1 && rec[13] == 0, "record: drop counters saturate at 0xFFFF");
}

int main(void) {
    test_record_layout();
    size_t n = sizeof(lh2_fixtures) / sizeof(lh2_fixtures[0]);
    for (size_t i = 0; i < n; i++) {
        test_fixture(&lh2_fixtures[i]);
    }
    printf("lh2_replay: %d passed, %d failed\n", _passed, _failed);
    return _failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
