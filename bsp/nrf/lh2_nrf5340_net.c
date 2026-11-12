/**
 * @file
 * @ingroup bsp_lh2
 *
 * @brief  nRF5340-net-specific definition of the "lh2" bsp module: no sensor, no capture
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include "gpio.h"
#include "lh2.h"

//=========================== public ===========================================

bool db_lh2_capture_init(const gpio_t *gpio_d, const gpio_t *gpio_e) {
    (void)gpio_d;
    (void)gpio_e;
    return false;
}

void db_lh2_capture_start(void) {
}

void db_lh2_capture_stop(void) {
}

bool db_lh2_capture_pop(db_lh2_capture_t *out) {
    (void)out;
    return false;
}

void db_lh2_capture_flush(void) {
}

uint32_t db_lh2_capture_dropped(void) {
    return 0;
}

uint32_t db_lh2_capture_now_us(void) {
    return 0;
}

void db_lh2_handle_isr(void) {
}
