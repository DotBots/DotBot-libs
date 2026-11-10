/**
 * @file
 * @brief  libFuzzer harness of drv/lh2_decoder and drv/lh2_select
 *
 * The input is a run of 69-byte records: a capture's 64 bytes, its 32-bit
 * stamp, little-endian, and a byte whose low bits say whether to read the
 * pairs after it (as the swarmit glue does every 100 ms) and how far the time
 * passed in runs ahead of the stamp. After every record the decoder's
 * invariants must hold; any violation aborts, which libFuzzer reports.
 *
 * @copyright Inria, 2026
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "lh2_decoder.h"
#include "lh2_geometry.h"
#include "lh2_select.h"

#define RECORD (DB_LH2_CAPTURE_BYTES + 5U)

static void _invariants(const db_lh2_decoder_t *dec, db_lh2_decode_result_t result) {
    if ((unsigned)result >= DB_LH2_DECODE_RESULTS) {
        abort();
    }
    for (uint8_t sweep = 0; sweep < DB_LH2_SWEEPS; sweep++) {
        for (uint8_t station = 0; station < DB_LH2_STATIONS; station++) {
            const db_lh2_slot_t *slot = &dec->slot[sweep][station];
            if (!slot->ready) {
                continue;
            }
            // a ready slot holds a count under one revolution, of a polynomial of its station
            if (slot->count >= db_lh2_period(station) / DB_LH2_PERIOD_TICKS_PER_COUNT || (slot->polynomial >> 1) != station) {
                abort();
            }
        }
    }
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
    static db_lh2_decoder_t     dec;
    static db_lh2_select_conf_t conf;
    db_lh2_decoder_init(&dec);
    memset(&conf, 0, sizeof(conf));
    float    h[DB_LH2_SELECT_STATIONS][3][3];
    uint32_t rects[DB_LH2_SELECT_STATIONS][DB_LH2_SELECT_RECT_LEN];
    memset(rects, 0xFF, sizeof(rects));
    for (uint8_t s = 0; s < DB_LH2_SELECT_STATIONS; s++) {
        float m[3][3] = { { 1500, 0, 1000 }, { 0, 1500, 1000 }, { 0.01f * s, 0, 1 } };
        memcpy(h[s], m, sizeof(m));
    }
    db_lh2_select_conf_load(&conf, 0xFFFF, (const float(*)[3][3])h, (const uint32_t(*)[DB_LH2_SELECT_RECT_LEN])rects);
    db_lh2_lines_t lines;
    db_lh2_lines_init(&lines);
    db_lh2_select_state_t state;
    db_lh2_select_state_init(&state);

    for (size_t off = 0; off + RECORD <= size; off += RECORD) {
        db_lh2_capture_t cap;
        memcpy(cap.spi, &data[off], DB_LH2_CAPTURE_BYTES);
        const uint8_t *t = &data[off + DB_LH2_CAPTURE_BYTES];
        cap.t_us         = (uint32_t)t[0] | (uint32_t)t[1] << 8 | (uint32_t)t[2] << 16 | (uint32_t)t[3] << 24;
        uint8_t  ctl     = t[4];
        uint32_t now_us  = cap.t_us + (uint32_t)(ctl >> 1) * 40000U;
        _invariants(&dec, db_lh2_decoder_feed(&dec, &cap, now_us));
        if (ctl & 1U) {
            db_lh2_pair_t pairs[DB_LH2_STATIONS];
            db_lh2_fix_t  fix;
            uint8_t       n = db_lh2_select_take_pairs(&dec, pairs, DB_LH2_STATIONS);
            if (db_lh2_select_fix(&conf, &state, pairs, n, &fix, &lines) && !db_lh2_select_rect_contains(conf.rect_mm[fix.station], fix.x_mm, fix.y_mm)) {
                abort();
            }
            db_lh2_floor_line_t out[DB_LH2_SELECT_LINES_MAX];
            if (db_lh2_lines_drain(&lines, out, DB_LH2_SELECT_LINES_MAX) % 2 != 0) {
                abort();
            }
        }
    }
    return 0;
}
