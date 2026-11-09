/**
 * @file
 * @brief  Host test of the fleet API wasm/dotbot_control_wasm.c exports, built natively
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dotbot_control_wasm.h"
#include "geometry.h"

//=========================== harness ==========================================

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

#define ROBOTS (5U)

//=========================== tests ============================================

static void test_seed(void) {
    CHECK(fleet_init(ROBOTS) == 0, "seed: fleet of %u", ROBOTS);
    fleet_seed(2, 1200.0f, 800.0f, -45.0f);
    fleet_seed(ROBOTS, 0.0f, 0.0f, 0.0f);  // no such robot: ignored

    db_control_report_t *reports = fleet_report_buffer();
    fleet_reports(reports);
    CHECK(reports[2].estimator_status == DB_POSE_ESTIMATOR_TRACKING && reports[2].direction == -45, "seed: robot 2 tracks facing -45, %d", reports[2].direction);
    CHECK(reports[2].axle_x == 1200 && reports[2].axle_y == 800, "seed: robot 2 at its axle, %u %u", reports[2].axle_x, reports[2].axle_y);
    CHECK(reports[1].direction == DB_CONTROL_DIRECTION_INVALID && reports[3].direction == DB_CONTROL_DIRECTION_INVALID, "seed: its neighbours have no heading");
}

static void test_advertisements(void) {
    CHECK(fleet_init(ROBOTS) == 0, "advertisements: fleet of %u", ROBOTS);
    fleet_seed(3, 1000.0f, 1000.0f, 0.0f);
    db_control_input_t  *inputs  = fleet_inputs();
    db_control_output_t *outputs = fleet_outputs();
    uint16_t            *battery = fleet_battery_buffer();
    uint8_t             *buffer  = fleet_advertisements_buffer();
    for (uint32_t i = 0; i < ROBOTS; i++) {
        battery[i] = (uint16_t)(3000 + i);
    }

    // Robots 1 and 3 reach their first advertisement on this step, the rest not
    memset(inputs, 0, ROBOTS * sizeof(*inputs));
    for (uint32_t i = 0; i < ROBOTS; i++) {
        inputs[i].elapsed_ticks = (i == 1 || i == 3) ? 50 : 1;
        inputs[i].counts_left   = (int32_t)i;
    }
    fleet_step(inputs, outputs);
    CHECK(outputs[1].advertise && outputs[3].advertise && !outputs[0].advertise, "advertisements: 1 and 3 are due");

    uint32_t n = fleet_advertisements(battery, buffer);
    CHECK(n == 2, "advertisements: two encoded, %u", n);
    uint32_t index[2];
    memcpy(index, buffer, sizeof(index));
    CHECK(index[0] == 1 && index[1] == 3, "advertisements: indices in order, %u %u", index[0], index[1]);
    const uint8_t *packets = buffer + n * sizeof(uint32_t);
    for (uint32_t k = 0; k < 2; k++) {
        const uint8_t *packet = packets + k * DB_CONTROL_ADVERTISEMENT_BYTES;
        uint16_t       level;
        int32_t        encoder_left;
        memcpy(&level, &packet[13], sizeof(level));
        memcpy(&encoder_left, &packet[18], sizeof(encoder_left));
        CHECK(packet[0] == DB_PROTOCOL_DOTBOT_ADVERTISEMENT, "advertisements: packet %u is an advertisement", k);
        CHECK(level == 3000 + index[k], "advertisements: packet %u carries its robot's battery, %u", k, level);
        CHECK(encoder_left == (int32_t)index[k], "advertisements: packet %u carries its robot's counts, %d", k, encoder_left);
    }
    int16_t direction;
    memcpy(&direction, &packets[DB_CONTROL_ADVERTISEMENT_BYTES + 3], sizeof(direction));
    CHECK(direction == 0, "advertisements: the seeded robot advertises its heading, %d", direction);

    CHECK(fleet_advertisements(battery, buffer) == 0, "advertisements: a second call finds none due");
    uint8_t single[DB_CONTROL_ADVERTISEMENT_BYTES];
    fleet_advertisement(1, 3001, single);
    int32_t encoder_left;
    memcpy(&encoder_left, &single[18], sizeof(encoder_left));
    CHECK(encoder_left == 0, "advertisements: the batch started robot 1's encoder deltas over, %d", encoder_left);
}

static void test_geometry(void) {
    CHECK(sizeof_geometry() == 11 * sizeof(float), "geometry: eleven floats, %u bytes", sizeof_geometry());
    const fleet_geometry_t *g = geometry();
    CHECK(g->wheel_diameter_mm == DB_WHEEL_DIAMETER && g->track_mm == DB_TRACK && g->mm_per_count == DB_MM_PER_COUNT, "geometry: wheel, track, mm per count");
    CHECK(g->lever_arm_effective_mm == DB_LH2_LEVER_ARM_EFFECTIVE && g->track_effective_arc_ratio == DB_TRACK_EFFECTIVE_ARC_RATIO, "geometry: effective lever arm, arc ratio");
}

static void test_fix_due(void) {
    CHECK(fleet_init(ROBOTS) == 0, "fix due: fleet of %u", ROBOTS);
    db_control_input_t  *inputs  = fleet_inputs();
    db_control_output_t *outputs = fleet_outputs();
    uint8_t             *mask    = fleet_fix_due_buffer();
    uint32_t             due     = 0;
    uint32_t             reads   = 0;
    for (uint32_t i = 0; i < ROBOTS; i++) {
        inputs[i] = (db_control_input_t){ .elapsed_ticks = 1 };
    }
    inputs[1].elapsed_ticks = 3;  // out of step with the others from here on
    fleet_step(inputs, outputs);
    inputs[1].elapsed_ticks = 1;
    for (uint32_t tick = 0; tick < 100; tick++) {
        uint8_t left[ROBOTS];
        memcpy(left, mask, sizeof(left));
        uint32_t n = fleet_fix_due(1, mask);
        CHECK(memcmp(left, mask, sizeof(left)) == 0, "fix due: the step left the same mask, tick %u", tick);
        uint32_t k = 0;
        for (uint32_t i = 0; i < ROBOTS; i++) {
            k += mask[i];
        }
        CHECK(n == k, "fix due: the count %u matches the mask %u", n, k);
        CHECK(mask[0] == mask[2], "fix due: robots in step agree");
        reads += mask[0];
        due += n;
        fleet_step(inputs, outputs);
    }
    CHECK(reads == 10, "fix due: one fix read in ten ticks, %u in 100", reads);
    CHECK(due == 10 * ROBOTS, "fix due: every robot due once per ten ticks, %u", due);
}

static void test_layout(void) {
    const uint32_t *offsets = layout_offsets();
    CHECK(layout_field_count() == 6 + 7 + 28 + 11, "layout: %u fields", layout_field_count());
    CHECK(offsets[0] == 0 && offsets[5] == 20, "layout: input's last field at %u", offsets[5]);
    CHECK(offsets[6 + 7 + 11] == 44, "layout: report's direction at %u", offsets[6 + 7 + 11]);
    CHECK(offsets[layout_field_count() - 1] == 40, "layout: geometry's last field at %u", offsets[layout_field_count() - 1]);
}

int main(void) {
    test_seed();
    test_advertisements();
    test_geometry();
    test_fix_due();
    test_layout();
    printf("dotbot_control fleet: %d passed, %d failed\n", _passed, _failed);
    return _failed ? 1 : 0;
}
