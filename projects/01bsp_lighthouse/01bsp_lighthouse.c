/**
 * @file
 * @ingroup samples_bsp
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to interface with the lighthouse v2 chip in the DotBot board.
 *
 * Load this program on your board. LED should blink blue when it receives a valid lighthouse 2 signal.
 *
 * @date 2022
 *
 * @copyright Inria, 2022
 *
 */
#include <stdio.h>
#include <nrf.h>
#include "board.h"
#include "board_config.h"
#include "lh2.h"

//=========================== defines ==========================================

//=========================== variables ========================================

static db_lh2_t _lh2;

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();

    // Initialize the LH2
    if (!db_lh2_init(&_lh2, &db_lh2_d, &db_lh2_e)) {
        // LH2 initialization failed
        puts("LH2 initialization failed");
        return -1;
    }
    puts("LH2 initialized with success");
    db_lh2_start();

    while (1) {
        // wait until something happens e.g. an SPI interrupt
        __WFE();

        // the location function has to be running all the time
        db_lh2_process_location(&_lh2);
        for (uint8_t bs_index = 0; bs_index < LH2_BASESTATION_COUNT; bs_index++) {
            if (_lh2.data_ready[0][bs_index] == DB_LH2_PROCESSED_DATA_AVAILABLE && _lh2.data_ready[1][bs_index] == DB_LH2_PROCESSED_DATA_AVAILABLE) {
                printf("BS idx: %d - counts: %u:%u\n", bs_index, _lh2.locations[0][bs_index].lfsr_counts, _lh2.locations[1][bs_index].lfsr_counts);
            }
        }
    }
}
