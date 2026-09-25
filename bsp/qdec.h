#ifndef __QDEC_H
#define __QDEC_H

/**
 * @defgroup    bsp_qdec    QDEC
 * @ingroup     bsp
 * @brief       Read the QDEC peripherals counter
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2023
 * @}
 */

#include <stdint.h>
#include <nrf.h>
#include "gpio.h"

//=========================== defines ==========================================

typedef void (*qdec_cb_t)(void *ctx);  ///< Callback function prototype, it is called on each qdec interrupt

typedef uint8_t qdec_t;  ///< QDEC peripheral index

/// QDEC pin configuration
typedef struct {
    const gpio_t *pin_a;  ///< GPIO pin A
    const gpio_t *pin_b;  ///< GPIO pin B
} qdec_conf_t;

//============================ public ==========================================

/**
 * @brief   Initialize a QDEC
 *
 * @param[in]   qdec            Pointer to the qdec descriptor
 * @param[in]   conf            Pointer to the qdec configuration
 * @param[in]   callback        Function pointer that is called from QDEC ISR
 * @param[in]   ctx             Pointer to some context passed as parameter to the callback
 */
void db_qdec_init(qdec_t qdec, const qdec_conf_t *conf, qdec_cb_t callback, void *ctx);

/**
 * @brief   Read the QDEC accumulator
 *
 * @param[in]   qdec            Index of the QDEC peripheral to read
 */
int32_t db_qdec_read(qdec_t qdec);

/**
 * @brief   Read and clear the QDEC accumulator
 *
 * @param[in]   qdec            Index of the QDEC peripheral to read and reset
 */
int32_t db_qdec_read_and_clear(qdec_t qdec);

/**
 * @brief   Read and clear the QDEC accumulator and its double-transition count
 *
 * A double transition is two quadrature steps inside one sample period. The
 * peripheral cannot tell their direction, so it counts them apart from the
 * accumulator; a caller wanting every step credits them in the direction the
 * accumulator moved. The count is 4 bits wide, so read it before 15 build up.
 *
 * @param[in]   qdec            Index of the QDEC peripheral to read and reset
 * @param[out]  dbl             Double transitions since the last clear
 *
 * @return  the signed accumulator, as db_qdec_read_and_clear() returns it
 */
int32_t db_qdec_read_and_clear_dbl(qdec_t qdec, uint32_t *dbl);

#endif
