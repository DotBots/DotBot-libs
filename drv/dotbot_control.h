#ifndef __DOTBOT_CONTROL_H
#define __DOTBOT_CONTROL_H

/**
 * @defgroup    drv_dotbot_control    DotBot control core
 * @ingroup     drv
 * @brief       The DotBot app's control, hardware-free: commands, counts and fixes in, motor duty and an advertisement out
 *
 * Ties the wheel loop, the pose estimator and the steering together the way
 * the DotBot app drives them: which writer owns the motors (idle, raw duty,
 * wheel speeds, waypoints), the batch id dedup and the abort reason, the
 * deadman stop of direct driving, and the advertisement. All state lives in
 * one db_control_t, so a simulator can hold many robots in an array.
 *
 * The caller owns the hardware and the clock. On every scheduler tick it
 * passes the encoder counts since the previous tick and the newest LH2 fix,
 * and writes the motors when the output asks to. Commands go to
 * db_control_rx() before the tick they apply on. When the output asks for an
 * advertisement, db_control_advertisement() encodes it.
 *
 * Time is counted in scheduler ticks of DB_CONTROL_TICK_MS; nothing here reads
 * a clock.
 *
 * The input, output and report structs are fixed-width and laid out the same
 * on 32- and 64-bit targets, for callers across a language or wasm boundary.
 * DB_CONTROL_ABI_VERSION changes whenever any of them or a function here does.
 *
 * @{
 * @file
 * @copyright Inria, 2026
 * @}
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lh2_fusion.h"
#include "lh2_geometry.h"
#include "pose_estimator.h"
#include "protocol.h"
#include "steering.h"
#include "wheel_control.h"

//=========================== defines ==========================================

/// Version of the structs and functions below, for callers loading a built copy
#define DB_CONTROL_ABI_VERSION (3U)

/// Period of one scheduler tick
#define DB_CONTROL_TICK_MS (10U)

/// Largest command db_control_rx() accepts, type byte included: a full
/// waypoint batch with its trailer and headings
#define DB_CONTROL_RX_MAX_BYTES (1U + sizeof(uint16_t) + 1U + DB_MAX_WAYPOINTS * (sizeof(protocol_lh2_location_t) + sizeof(int16_t)) + sizeof(protocol_lh2_waypoints_trailer_t))

/// Length of the advertisement db_control_advertisement() writes
#define DB_CONTROL_ADVERTISEMENT_BYTES (43U)

/// Floor lines one tick can take: four stations, two sweeps each
#define DB_CONTROL_LINES_MAX (8U)

/// The advertisement's heading while the estimator has none
#define DB_CONTROL_DIRECTION_INVALID (-1000)

/// Who writes the motors. Exactly one writer per mode.
typedef enum {
    DB_CONTROL_DRIVE_IDLE,      ///< Nothing commanded; the wheel loop at zero brakes a turning wheel, else coasts
    DB_CONTROL_DRIVE_RAW,       ///< MOVE_RAW writes duty directly and the wheel loop is off
    DB_CONTROL_DRIVE_VELOCITY,  ///< The wheel loop is the only writer
    DB_CONTROL_DRIVE_WAYPOINT,  ///< The steering sets the wheel loop's setpoints, or holds the motors braked
} db_control_drive_mode_t;

/// Gains and limits; must outlive every db_control_t bound to it
typedef struct {
    db_wheel_control_conf_t  wheel;          ///< both wheels' loops
    db_pose_estimator_conf_t estimator;      ///< pose estimator
    db_steering_conf_t       steering;       ///< steering along waypoints
    uint32_t                 deadman_ticks;  ///< raw and velocity driving stop past this long without a command
} db_control_conf_t;

/// One tick's inputs
typedef struct {
    int32_t  counts_left;    ///< Encoder counts since the previous tick, doubles credited
    int32_t  counts_right;   ///< Encoder counts since the previous tick, doubles credited
    uint32_t fix_sequence;   ///< Sequence of the newest LH2 solve; a change is a new fix, 0 before the first
    uint32_t fix_x;          ///< Photodiode of that solve, mm
    uint32_t fix_y;          ///< Photodiode of that solve, mm
    uint32_t elapsed_ticks;  ///< Ticks since the previous call, more than 1 after a dropped backlog; 0 is taken as 1
} db_control_input_t;

/// One tick's outputs
typedef struct {
    int8_t pwm_left;     ///< Duty, -100 to 100
    int8_t pwm_right;    ///< Duty, -100 to 100
    bool   brake_left;   ///< Short the left motor, ignoring its duty
    bool   brake_right;  ///< Short the right motor, ignoring its duty
    bool   write;        ///< Write the four fields above; else leave the motors as they are
    bool   advertise;    ///< An advertisement is due
    bool   reserved[2];  ///< Always false
} db_control_output_t;

/// What the robot reports, the advertisement's fields and more
typedef struct {
    float    axle_x_mm;         ///< Estimated axle midpoint, whatever the estimator's status
    float    axle_y_mm;         ///< Estimated axle midpoint, whatever the estimator's status
    float    heading_deg;       ///< Estimated heading, 0 facing +y, clockwise positive
    float    max_speed_mm_s;    ///< Cruise speed in force
    uint32_t sensor_x;          ///< Photodiode as advertised: the estimator's while it has one, else the last fix, mm
    uint32_t sensor_y;          ///< Photodiode as advertised, mm
    uint32_t waypoint_x;        ///< Point being driven to, 0 while the steering is idle, mm
    uint32_t waypoint_y;        ///< Point being driven to, 0 while the steering is idle, mm
    uint32_t encoder_left;      ///< Counts since init, wrapping
    uint32_t encoder_right;     ///< Counts since init, wrapping
    uint32_t fix_sequence;      ///< Sequence of the last fix read
    int16_t  direction;         ///< Heading as advertised, deg, DB_CONTROL_DIRECTION_INVALID without one
    uint16_t axle_x;            ///< Axle as advertised, mm, DB_AXLE_UNKNOWN unless tracking
    uint16_t axle_y;            ///< Axle as advertised, mm, DB_AXLE_UNKNOWN unless tracking
    int8_t   pwm_left;          ///< Last duty written, 0 while braked
    int8_t   pwm_right;         ///< Last duty written, 0 while braked
    uint8_t  brake_left;        ///< Last brake written
    uint8_t  brake_right;       ///< Last brake written
    uint8_t  control_mode;      ///< protocol_control_mode_t: auto while a batch is active
    uint8_t  drive_mode;        ///< db_control_drive_mode_t
    uint8_t  steering_state;    ///< db_steering_state_t
    uint8_t  estimator_status;  ///< db_pose_estimator_status_t
    uint8_t  waypoint_index;    ///< Point being driven to; the count once arrived
    uint8_t  waypoint_count;    ///< Points in the batch
    uint8_t  batch_id;          ///< Of the last batch accepted, an empty one included, 0 for none
    uint8_t  status;            ///< protocol_waypoints_status_t
    uint8_t  reason;            ///< protocol_waypoints_fail_t or protocol_waypoints_abort_t, by status; else 0
    uint8_t  max_speed_10mm;    ///< Cruise speed as advertised, in units of 10 mm/s
} db_control_report_t;

/// One robot's control state
typedef struct {
    const db_control_conf_t   *conf;                         ///< gains, not owned
    db_wheel_control_t         wheel_left;                   ///< left wheel loop
    db_wheel_control_t         wheel_right;                  ///< right wheel loop
    db_pose_estimator_t        estimator;                    ///< pose estimator
    db_steering_t              steering;                     ///< steering along waypoints
    db_lh2_fusion_t            fusion;                       ///< per-station health of the LH2 sweeps fused
    db_lh2_floor_line_t        lines[DB_CONTROL_LINES_MAX];  ///< staged for the next tick
    uint8_t                    line_count;                   ///< lines staged
    db_control_drive_mode_t    drive_mode;                   ///< which writer owns the motors
    bool                       steering_brake;               ///< the steering holds the motors braked
    uint8_t                    batch_id;                     ///< of the last batch accepted, 0 for none
    protocol_waypoints_abort_t abort_reason;                 ///< what stopped the last batch
    uint32_t                   tick;                         ///< ticks since init
    uint32_t                   tick_position;                ///< tick of the last fix read
    uint32_t                   tick_steering;                ///< tick of the last steering step
    uint32_t                   tick_timeout;                 ///< tick of the last deadman check
    uint32_t                   tick_advert;                  ///< tick of the last advertisement
    uint32_t                   advert_period_ticks;          ///< ticks between advertisements
    uint32_t                   last_command_tick;            ///< tick before the last direct or waypoint command
    uint32_t                   fix_sequence;                 ///< sequence of the last fix read
    bool                       has_position;                 ///< a fix in bounds has been read
    uint32_t                   position_x;                   ///< last fix in bounds, mm
    uint32_t                   position_y;                   ///< last fix in bounds, mm
    uint32_t                   encoder_left;                 ///< counts since init, wrapping
    uint32_t                   encoder_right;                ///< counts since init, wrapping
    uint32_t                   advert_encoder_left;          ///< encoder_left at the last advertisement
    uint32_t                   advert_encoder_right;         ///< encoder_right at the last advertisement
    db_control_output_t        pending;                      ///< a motor write a command made, delivered by the next tick
    int8_t                     pwm_left;                     ///< last duty written, 0 while braked
    int8_t                     pwm_right;                    ///< last duty written, 0 while braked
    bool                       brake_left;                   ///< last brake written
    bool                       brake_right;                  ///< last brake written
} db_control_t;

/// The DotBot v3 gains and limits the app runs with
extern const db_control_conf_t db_control_default_conf;

//=========================== prototypes =======================================

/**
 * @brief   Bind a robot to its gains, idle, with the pose unknown
 *
 * @param[out]  control     Robot state
 * @param[in]   conf        Gains, which must outlive the robot
 */
void db_control_init(db_control_t *control, const db_control_conf_t *conf);

/**
 * @brief   Set the pose outright, heading known, and start the estimator TRACKING
 *
 * An entry point for simulators and tests.
 *
 * @param[in,out]   control         Robot state
 * @param[in]       axle_x_mm       Axle midpoint
 * @param[in]       axle_y_mm       Axle midpoint
 * @param[in]       heading_deg     Heading, 0 facing +y, clockwise positive
 */
void db_control_seed(db_control_t *control, float axle_x_mm, float axle_y_mm, float heading_deg);

/**
 * @brief   Apply one command, received since the previous tick
 *
 * Handles MOVE_RAW, WHEEL_VELOCITY, LH2_WAYPOINTS, MAX_SPEED and CONTROL_MODE;
 * ignores every other type, and a command longer than DB_CONTROL_RX_MAX_BYTES.
 *
 * @param[in,out]   control     Robot state
 * @param[in]       packet      Type byte, then the payload
 * @param[in]       length      Bytes in packet
 */
void db_control_rx(db_control_t *control, const uint8_t *packet, size_t length);

/**
 * @brief   Whether the next db_control_tick() reads the fix
 *
 * The firmware runs the LH2 solve only then; a caller that always has a fix
 * need not ask.
 *
 * @param[in]   control         Robot state
 * @param[in]   elapsed_ticks   What the next input will carry
 *
 * @return  true if the next tick reads the fix
 */
bool db_control_fix_due(const db_control_t *control, uint32_t elapsed_ticks);

/**
 * @brief   Stage the LH2 floor lines read with this tick's fix
 *
 * The next db_control_tick() fuses them into the estimator after its predict,
 * while the estimator tracks. When every one is accepted, that tick's fix
 * feeds the steering and the advertisement but not the estimator, since it
 * comes from the same sweeps; when any is rejected, the fix is used as
 * without lines, so its gate and the kidnap check still see the jump. Lines beyond
 * DB_CONTROL_LINES_MAX are dropped, and a second call before the tick
 * replaces the first.
 *
 * @param[in,out]   control     Robot state
 * @param[in]       lines       Floor lines
 * @param[in]       count       Lines in lines
 */
void db_control_lines(db_control_t *control, const db_lh2_floor_line_t *lines, uint8_t count);

/**
 * @brief   Run one scheduler tick
 *
 * @param[in,out]   control     Robot state
 * @param[in]       in          Counts and fix
 * @param[out]      out         Motor write and what else is due
 */
void db_control_tick(db_control_t *control, const db_control_input_t *in, db_control_output_t *out);

/**
 * @brief   Set the advertisement period from the node's minimum TX interval
 *
 * Takes effect from the next advertisement check.
 *
 * @param[in,out]   control             Robot state
 * @param[in]       min_tx_interval_us  0 while not joined
 */
void db_control_set_min_tx_interval(db_control_t *control, uint32_t min_tx_interval_us);

/**
 * @brief   Report the robot's state
 *
 * @param[in]   control     Robot state
 * @param[out]  report      Filled in full
 */
void db_control_report(const db_control_t *control, db_control_report_t *report);

/**
 * @brief   Encode DB_PROTOCOL_DOTBOT_ADVERTISEMENT with its waypoint report
 *
 * Starts the encoder deltas it carries over from here.
 *
 * @param[in,out]   control         Robot state
 * @param[in]       battery_level   Battery, as the node reads it
 * @param[out]      buffer          At least DB_CONTROL_ADVERTISEMENT_BYTES
 *
 * @return  bytes written
 */
size_t db_control_advertisement(db_control_t *control, uint16_t battery_level, uint8_t *buffer);

#endif
