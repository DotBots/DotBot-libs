/**
 * @file
 * @ingroup samples_drv
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @brief Simple application used to read and print the battery level
 *
 * @copyright Inria, 2025
 *
 */
#include <nrf.h>
#include <stdio.h>
#include "timer.h"
#include "battery.h"

int main(void) {
    db_timer_init(0);
    db_battery_level_init();

    while (1) {
        uint16_t battery_level = db_battery_level_read();
        printf("Battery level: %d mV\n", battery_level);
        db_timer_delay_ms(0, 1000);
    }
}
