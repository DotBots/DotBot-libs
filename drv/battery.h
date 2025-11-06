#ifndef __BATTERY_H
#define __BATTERY_H

/**
 * @defgroup    drv_battery  Battery level measurement functions
 * @ingroup     drv
 * @brief       Functions for measuring battery level
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2025
 * @}
 */

#include <stdint.h>

// For reading the battery level
#if defined(BOARD_DOTBOT_V3)
#define DB_BATTERY_LEVEL_PIN (DB_SAADC_INPUT_AIN1)
#else
#define DB_BATTERY_LEVEL_PIN (DB_SAADC_INPUT_VDD)
#endif

/// Maximum battery level in millivolts
#define DB_BATTERY_LEVEL_MAX_MV (3000)

/**
 * @brief Initialize the battery level measurement module
 */
void db_battery_level_init(void);

/**
 * @brief Read the battery level in millivolts
 *
 * @return Battery level in millivolts
 */
uint16_t db_battery_level_read(void);

#endif  // __BATTERY_H
