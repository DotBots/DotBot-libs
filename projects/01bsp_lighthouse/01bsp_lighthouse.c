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
#include "lh2_decoder.h"

//=========================== defines ==========================================

//=========================== variables ========================================

static db_lh2_decoder_t _decoder;

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();

    // Initialize the LH2
    db_lh2_decoder_init(&_decoder);
    if (!db_lh2_capture_init(&db_lh2_d, &db_lh2_e)) {
        // LH2 initialization failed
        puts("LH2 initialization failed");
        return -1;
    }
    puts("LH2 initialized with success");
    db_lh2_capture_start();

    while (1) {
        // wait until something happens e.g. an SPI interrupt
        __WFE();

        // decode one queued capture per wake-up
        db_lh2_capture_t capture;
        if (db_lh2_capture_pop(&capture)) {
            db_lh2_decoder_feed(&_decoder, &capture, db_lh2_capture_now_us());
        }
        for (uint8_t bs_index = 0; bs_index < DB_LH2_STATIONS; bs_index++) {
            db_lh2_pair_t pair;
            if (db_lh2_decoder_take_pair(&_decoder, bs_index, &pair)) {
                printf("BS idx: %d - counts: %u:%u\n", bs_index, pair.count[0], pair.count[1]);
            }
        }
    }
}
