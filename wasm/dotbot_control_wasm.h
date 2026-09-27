#ifndef __DOTBOT_CONTROL_WASM_H
#define __DOTBOT_CONTROL_WASM_H

/**
 * @file
 * @brief  The fleet API wasm/dotbot_control_wasm.c exports, for the host tests that build it natively
 *
 * @copyright Inria, 2026
 */

#include <stdint.h>

#include "dotbot_control.h"

/// drv/geometry.h for the board the core is built for, all floats, mm and deg
typedef struct {
    float wheel_diameter_mm;          ///< DB_WHEEL_DIAMETER
    float track_mm;                   ///< DB_TRACK
    float encoder_cpr;                ///< DB_ENCODER_CPR
    float gear_ratio;                 ///< DB_GEAR_RATIO
    float mm_per_count;               ///< DB_MM_PER_COUNT
    float lever_arm_mm;               ///< DB_LH2_LEVER_ARM
    float lever_angle_deg;            ///< DB_LH2_LEVER_ANGLE
    float lever_arm_effective_mm;     ///< DB_LH2_LEVER_ARM_EFFECTIVE
    float track_effective_mm;         ///< DB_TRACK_EFFECTIVE
    float track_effective_arc_mm;     ///< DB_TRACK_EFFECTIVE_ARC
    float track_effective_arc_ratio;  ///< DB_TRACK_EFFECTIVE_ARC_RATIO
} fleet_geometry_t;

uint32_t                abi_version(void);
uint32_t                sizeof_state(void);
uint32_t                sizeof_input(void);
uint32_t                sizeof_output(void);
uint32_t                sizeof_report(void);
uint32_t                rx_max_bytes(void);
uint32_t                advertisement_bytes(void);
uint32_t                sizeof_geometry(void);
const fleet_geometry_t *geometry(void);
int32_t                 fleet_init(uint32_t count);
uint32_t                fleet_count(void);
db_control_input_t     *fleet_inputs(void);
db_control_output_t    *fleet_outputs(void);
db_control_report_t    *fleet_report_buffer(void);
uint16_t               *fleet_battery_buffer(void);
uint8_t                *fleet_advertisements_buffer(void);
uint8_t                *fleet_rx_buffer(void);
uint8_t                *fleet_advertisement_buffer(void);
void                    fleet_rx(uint32_t index, const uint8_t *packet, uint32_t length);
void                    fleet_step(const db_control_input_t *inputs, db_control_output_t *outputs);
void                    fleet_seed(uint32_t index, float axle_x_mm, float axle_y_mm, float heading_deg);
void                    fleet_reports(db_control_report_t *reports);
uint32_t                fleet_advertisement(uint32_t index, uint32_t battery_level, uint8_t *buffer);
uint32_t                fleet_advertisements(const uint16_t *battery, uint8_t *buffer);
void                    fleet_set_min_tx_interval(uint32_t index, uint32_t min_tx_interval_us);

#endif
