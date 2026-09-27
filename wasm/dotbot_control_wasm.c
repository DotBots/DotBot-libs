/**
 * @file
 * @brief  WebAssembly exports of drv/dotbot_control: a fleet of robots stepped in one call
 *
 * The host writes one db_control_input_t per robot into fleet_inputs(), calls
 * fleet_step() once per tick, and reads one db_control_output_t per robot back
 * from fleet_outputs(). Commands go through fleet_rx_buffer() and fleet_rx().
 * fleet_advertisements() encodes that tick's advertisements in one call.
 *
 * Built natively too, for the host tests, where nothing is exported.
 *
 * @copyright Inria, 2026
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "dotbot_control_wasm.h"
#include "geometry.h"

#if defined(__wasm__)
#define EXPORT(name) __attribute__((export_name(#name)))
#else
#define EXPORT(name)
#endif

_Static_assert(sizeof(fleet_geometry_t) == 44, "fleet_geometry_t is an ABI");

static const fleet_geometry_t _geometry = {
    .wheel_diameter_mm         = DB_WHEEL_DIAMETER,
    .track_mm                  = DB_TRACK,
    .encoder_cpr               = DB_ENCODER_CPR,
    .gear_ratio                = DB_GEAR_RATIO,
    .mm_per_count              = DB_MM_PER_COUNT,
    .lever_arm_mm              = DB_LH2_LEVER_ARM,
    .lever_angle_deg           = DB_LH2_LEVER_ANGLE,
    .lever_arm_effective_mm    = DB_LH2_LEVER_ARM_EFFECTIVE,
    .track_effective_mm        = DB_TRACK_EFFECTIVE,
    .track_effective_arc_mm    = DB_TRACK_EFFECTIVE_ARC,
    .track_effective_arc_ratio = DB_TRACK_EFFECTIVE_ARC_RATIO,
};

static db_control_t        *_robots;
static db_control_input_t  *_inputs;
static db_control_output_t *_outputs;
static db_control_report_t *_reports;
static uint8_t             *_advertise;  ///< per robot, the last step asked for an advertisement
static uint16_t            *_battery;
static uint8_t             *_advertisements;
static uint32_t             _count;
static uint8_t              _rx_buffer[DB_CONTROL_RX_MAX_BYTES];
static uint8_t              _advertisement[DB_CONTROL_ADVERTISEMENT_BYTES];

EXPORT(abi_version)
uint32_t abi_version(void) {
    return DB_CONTROL_ABI_VERSION;
}

EXPORT(sizeof_state)
uint32_t sizeof_state(void) {
    return sizeof(db_control_t);
}

EXPORT(sizeof_input)
uint32_t sizeof_input(void) {
    return sizeof(db_control_input_t);
}

EXPORT(sizeof_output)
uint32_t sizeof_output(void) {
    return sizeof(db_control_output_t);
}

EXPORT(sizeof_report)
uint32_t sizeof_report(void) {
    return sizeof(db_control_report_t);
}

EXPORT(rx_max_bytes)
uint32_t rx_max_bytes(void) {
    return DB_CONTROL_RX_MAX_BYTES;
}

EXPORT(advertisement_bytes)
uint32_t advertisement_bytes(void) {
    return DB_CONTROL_ADVERTISEMENT_BYTES;
}

EXPORT(sizeof_geometry)
uint32_t sizeof_geometry(void) {
    return sizeof(fleet_geometry_t);
}

EXPORT(geometry)
const fleet_geometry_t *geometry(void) {
    return &_geometry;
}

/// Replaces the fleet with count robots, idle and without a pose; 0 on success
EXPORT(fleet_init)
int32_t fleet_init(uint32_t count) {
    free(_robots);
    free(_inputs);
    free(_outputs);
    free(_reports);
    free(_advertise);
    free(_battery);
    free(_advertisements);
    _count          = 0;
    _robots         = calloc(count, sizeof(*_robots));
    _inputs         = calloc(count, sizeof(*_inputs));
    _outputs        = calloc(count, sizeof(*_outputs));
    _reports        = calloc(count, sizeof(*_reports));
    _advertise      = calloc(count, sizeof(*_advertise));
    _battery        = calloc(count, sizeof(*_battery));
    _advertisements = calloc(count, sizeof(uint32_t) + DB_CONTROL_ADVERTISEMENT_BYTES);
    if (count > 0 && !(_robots && _inputs && _outputs && _reports && _advertise && _battery && _advertisements)) {
        return -1;
    }
    for (uint32_t i = 0; i < count; i++) {
        db_control_init(&_robots[i], &db_control_default_conf);
    }
    _count = count;
    return 0;
}

EXPORT(fleet_count)
uint32_t fleet_count(void) {
    return _count;
}

EXPORT(fleet_inputs)
db_control_input_t *fleet_inputs(void) {
    return _inputs;
}

EXPORT(fleet_outputs)
db_control_output_t *fleet_outputs(void) {
    return _outputs;
}

EXPORT(fleet_report_buffer)
db_control_report_t *fleet_report_buffer(void) {
    return _reports;
}

/// One uint16_t battery level per robot, for fleet_advertisements()
EXPORT(fleet_battery_buffer)
uint16_t *fleet_battery_buffer(void) {
    return _battery;
}

/// Room for every robot's advertisement: fleet_count() * (4 + advertisement_bytes())
EXPORT(fleet_advertisements_buffer)
uint8_t *fleet_advertisements_buffer(void) {
    return _advertisements;
}

EXPORT(fleet_rx_buffer)
uint8_t *fleet_rx_buffer(void) {
    return _rx_buffer;
}

EXPORT(fleet_advertisement_buffer)
uint8_t *fleet_advertisement_buffer(void) {
    return _advertisement;
}

EXPORT(fleet_rx)
void fleet_rx(uint32_t index, const uint8_t *packet, uint32_t length) {
    if (index < _count) {
        db_control_rx(&_robots[index], packet, length);
    }
}

/// One tick of every robot: inputs and outputs are arrays of fleet_count()
EXPORT(fleet_step)
void fleet_step(const db_control_input_t *inputs, db_control_output_t *outputs) {
    for (uint32_t i = 0; i < _count; i++) {
        db_control_tick(&_robots[i], &inputs[i], &outputs[i]);
        _advertise[i] = outputs[i].advertise;
    }
}

/// Sets robot index's pose outright, heading known: see db_control_seed()
EXPORT(fleet_seed)
void fleet_seed(uint32_t index, float axle_x_mm, float axle_y_mm, float heading_deg) {
    if (index < _count) {
        db_control_seed(&_robots[index], axle_x_mm, axle_y_mm, heading_deg);
    }
}

EXPORT(fleet_reports)
void fleet_reports(db_control_report_t *reports) {
    for (uint32_t i = 0; i < _count; i++) {
        db_control_report(&_robots[i], &reports[i]);
    }
}

/// Encodes robot index's advertisement into buffer; 0 for no such robot
EXPORT(fleet_advertisement)
uint32_t fleet_advertisement(uint32_t index, uint32_t battery_level, uint8_t *buffer) {
    if (index >= _count) {
        return 0;
    }
    return (uint32_t)db_control_advertisement(&_robots[index], (uint16_t)battery_level, buffer);
}

/**
 * Encodes the advertisement of every robot the last fleet_step() asked one of,
 * in index order, and clears those requests. battery holds one level per
 * robot. Returns n, the number encoded; buffer then holds n uint32_t robot
 * indices, then n advertisements of advertisement_bytes() each.
 */
EXPORT(fleet_advertisements)
uint32_t fleet_advertisements(const uint16_t *battery, uint8_t *buffer) {
    uint32_t n = 0;
    for (uint32_t i = 0; i < _count; i++) {
        n += _advertise[i];
    }
    uint8_t *packets = buffer + n * sizeof(uint32_t);
    uint32_t k       = 0;
    for (uint32_t i = 0; i < _count && k < n; i++) {
        if (!_advertise[i]) {
            continue;
        }
        _advertise[i] = 0;
        memcpy(buffer + k * sizeof(uint32_t), &i, sizeof(i));
        db_control_advertisement(&_robots[i], battery[i], packets + k * DB_CONTROL_ADVERTISEMENT_BYTES);
        k++;
    }
    return n;
}

EXPORT(fleet_set_min_tx_interval)
void fleet_set_min_tx_interval(uint32_t index, uint32_t min_tx_interval_us) {
    if (index < _count) {
        db_control_set_min_tx_interval(&_robots[index], min_tx_interval_us);
    }
}
