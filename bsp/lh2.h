#ifndef __LH2_H_
#define __LH2_H_

/**
 * @defgroup    bsp_lh2 LightHouse 2 support
 * @ingroup     bsp
 * @brief       Capture the LH2 sweeps of the TS4231
 *
 * The capture layer: it configures the TS4231, and on every falling edge of
 * its envelope line the SPIM reads 64 bytes of the data line; the SPIM's END
 * interrupt stamps the capture and queues it. Decoding is the caller's, with
 * drv/lh2_decoder:
 *
 *     db_lh2_capture_t cap;
 *     if (db_lh2_capture_pop(&cap)) {
 *         db_lh2_decoder_feed(&decoder, &cap, db_lh2_capture_now_us());
 *     }
 *
 * Built with DB_LH2_CAPTURE_RTT defined, every capture popped is also written
 * to RTT up-buffer 1 as a db_lh2_capture_record(), without blocking, for
 * tools/lh2_capture.py to turn into test fixtures. The build option costs
 * an 8 KiB buffer and is meant for a recording build only.
 *
 * @{
 * @file
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022-present
 * @}
 */

#include <stdint.h>
#include <stdbool.h>

#include "gpio.h"
#include "lh2_decoder.h"

//=========================== public ===========================================

/**
 * @brief Configure the TS4231 and the capture of its sweeps; captures start with db_lh2_capture_start()
 *
 * @param[in]   gpio_d  the TS4231 data line
 * @param[in]   gpio_e  the TS4231 envelope line
 * @return whether the TS4231 answered its configuration
 */
bool db_lh2_capture_init(const gpio_t *gpio_d, const gpio_t *gpio_e);

/// Arm the capture of the next envelope edge
void db_lh2_capture_start(void);

/// Disarm it; a capture already running still completes and is queued
void db_lh2_capture_stop(void);

/// Take the oldest queued capture, with the SPIM interrupt masked; false when none
bool db_lh2_capture_pop(db_lh2_capture_t *out);

/// Drop every queued capture
void db_lh2_capture_flush(void);

/// Captures overwritten in the queue before they were popped
uint32_t db_lh2_capture_dropped(void);

/// The time captures are stamped with, in microseconds
uint32_t db_lh2_capture_now_us(void);

/// Queue the capture that just completed and re-arm: the SPIM END interrupt's work
void db_lh2_handle_isr(void);

#endif /* __LH2_H_ */
