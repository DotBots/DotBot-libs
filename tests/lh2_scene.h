#ifndef __LH2_SCENE_H
#define __LH2_SCENE_H

/**
 * @file
 * @brief  Synthetic LH2 scenes: captures of a moving photodiode from several stations
 *
 * A station is a homography from its camera point to the floor, the inverse
 * of which turns a floor point into the two sweep counts (the inverse of
 * db_lh2_camera_point()), and a rotor running at its channel's period from a
 * chosen phase. Between two times, the photodiode moves in a straight line;
 * every sweep that crosses it in that window becomes a capture through
 * lh2_synth_capture(), stamped like the SPIM handler would.
 *
 * @copyright Inria, 2026
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lh2_synth.h"

/// One station of a scene, and the knobs that act across its captures
typedef struct {
    uint8_t station;          ///< lh_index 0..15
    float   h[3][3];          ///< camera point to floor mm, as a calibration stores it
    double  h_inv[3][3];      ///< floor mm to camera point, the inverse of h
    double  t0_us;            ///< the start of revolution 0, on the scene's clock
    double  dup_share;        ///< dup-late: the share of passes with a second capture
    double  dup_delay_us[2];  ///< dup-late: its delay after the first, uniform in [min, max]
    uint8_t dup_sweeps;       ///< dup-late: bit s set when sweep s can get one
    double  silent_us[2];     ///< drop: no capture from silent_us[0] to silent_us[1]
    double  latency_us;       ///< latency: added to each stamp, uniform in [0, latency_us]
} lh2_scene_station_t;

/**
 * @brief   A station looking down at the floor
 *
 * The homography maps a camera point (u, v) to k (cos a u - sin a v, sin a u + cos a v)
 * plus (cx, cy), with a small perspective term (pu, pv).
 *
 * @return  false when the homography cannot be inverted
 */
bool lh2_scene_station_init(lh2_scene_station_t *st, uint8_t station, double cx_mm, double cy_mm, double k_mm, double yaw_rad, double pu, double pv, double t0_us);

/// The camera point a floor point gives a station
void lh2_scene_camera_point(const lh2_scene_station_t *st, double x_mm, double y_mm, double cam[2]);

/**
 * @brief   The two sweep counts of a camera point, real-valued: the inverse of db_lh2_camera_point()
 *
 * @return  false when no sweep pair of one revolution sees the point
 */
bool lh2_scene_counts(uint8_t station, const double cam[2], double counts[2]);

/**
 * @brief   The captures of every station between two times, in arrival order
 *
 * @param[in]   stations    the scene's stations
 * @param[in]   n_stations  how many
 * @param[in]   from_us     the window's start on the scene's clock, inclusive
 * @param[in]   to_us       its end, exclusive
 * @param[in]   xy_from     the photodiode at from_us, mm
 * @param[in]   xy_to       the photodiode at to_us, mm; it moves in a straight line between
 * @param[in]   base_us     the 32-bit timer at the scene's time 0, so 2^32 falls where wanted
 * @param[out]  out         at most max events
 *
 * @return  the number of events; max when the window did not fit
 */
size_t lh2_scene_window(const lh2_scene_station_t *stations, uint8_t n_stations, double from_us, double to_us, const double xy_from[2], const double xy_to[2], uint64_t base_us, lh2_synth_rng_t *rng, lh2_synth_event_t *out, size_t max);

#endif
