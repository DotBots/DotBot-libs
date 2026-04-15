/**
 * @file
 * @ingroup bsp_qdec
 *
 * @brief  Target-specific definition of the "qdec" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#include "qdec_default.c"
#else
#include "qdec_stub.c"
#endif
