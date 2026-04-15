/**
 * @file
 * @ingroup bsp_qdec
 *
 * @brief  Stub definition of the "qdec" bsp module for platforms without
 *         an nRF5340 application core.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdint.h>

#include "qdec.h"

//=========================== public ===========================================

void db_qdec_init(qdec_t qdec, const qdec_conf_t *conf, qdec_cb_t callback, void *ctx) {
    (void)qdec;
    (void)conf;
    (void)callback;
    (void)ctx;
}

int32_t db_qdec_read(qdec_t qdec) {
    (void)qdec;
    return 0;
}

int32_t db_qdec_read_and_clear(qdec_t qdec) {
    (void)qdec;
    return 0;
}
