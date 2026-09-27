/**
 * @file
 * @brief  WebAssembly exports of drv/dotbot_control: a fleet of robots stepped in one call
 *
 * The host writes one db_control_input_t per robot into fleet_inputs(), calls
 * fleet_step() once per tick, and reads one db_control_output_t per robot back
 * from fleet_outputs(). Commands go through fleet_rx_buffer() and fleet_rx().
 *
 * @copyright Inria, 2026
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "dotbot_control.h"

#define EXPORT(name) __attribute__((export_name(#name)))

static db_control_t        *_robots;
static db_control_input_t  *_inputs;
static db_control_output_t *_outputs;
static db_control_report_t *_reports;
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

/// Replaces the fleet with count robots, idle and without a pose; 0 on success
EXPORT(fleet_init)
int32_t fleet_init(uint32_t count) {
    free(_robots);
    free(_inputs);
    free(_outputs);
    free(_reports);
    _count   = 0;
    _robots  = calloc(count, sizeof(*_robots));
    _inputs  = calloc(count, sizeof(*_inputs));
    _outputs = calloc(count, sizeof(*_outputs));
    _reports = calloc(count, sizeof(*_reports));
    if (count > 0 && !(_robots && _inputs && _outputs && _reports)) {
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

EXPORT(fleet_set_min_tx_interval)
void fleet_set_min_tx_interval(uint32_t index, uint32_t min_tx_interval_us) {
    if (index < _count) {
        db_control_set_min_tx_interval(&_robots[index], min_tx_interval_us);
    }
}
