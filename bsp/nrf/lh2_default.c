/**
 * @file
 * @ingroup bsp_lh2
 *
 * @brief  nRF52833-specific definition of the "lh2" bsp module.
 *
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <nrf.h>
#include <math.h>

#include "gpio.h"
#include "lh2.h"
#include "lh2_decoder.h"
#include "lh2_checkpoints.h"
#include "timer_hf.h"
#include "board_config.h"

//=========================== defines =========================================

#define SPIM_INTERRUPT_PRIORITY 2   ///< Interrupt priority, as high as it will go
#define SPI_BUFFER_SIZE         64  ///< Size of buffers used for SPI communications
#if defined(BOARD_DOTBOT_V3)
#define SPI_FAKE_SCK_PIN 7  ///< NOTE: SPIM needs an SCK pin to be defined, P1.6 is used because it's not an available pin in the BCM module.
#else
#define SPI_FAKE_SCK_PIN 6  ///< NOTE: SPIM needs an SCK pin to be defined, P1.6 is used because it's not an available pin in the BCM module.
#endif
#define SPI_FAKE_SCK_PORT                      1                                                              ///< NOTE: SPIM needs an SCK pin to be defined, P1.6 is used because it's not an available pin in the BCM module.
#define FUZZY_CHIP                             0xFF                                                           ///< not sure what this is about
#define LH2_LOCATION_ERROR_INDICATOR           0xFFFFFFFF                                                     ///< indicate the location value is false
#define LH2_POLYNOMIAL_ERROR_INDICATOR         0xFF                                                           ///< indicate the polynomial index is invalid
#define POLYNOMIAL_BIT_ERROR_INITIAL_THRESHOLD 4                                                              ///< initial threshold of polynomial error
#define LH2_BUFFER_SIZE                        10                                                             ///< Amount of lh2 frames the buffer can contain
#define GPIOTE_CH_IN_ENV_HiToLo                1                                                              ///< falling edge gpio channel
#define GPIOTE_CH_IN_ENV_LoToHi                2                                                              ///< rising edge gpio channel
#define PPI_SPI_START_CHAN                     2                                                              ///< PPI channel for starting the GPIOTE to SPI capture ppi
#define PPI_SPI_GROUP                          0                                                              ///< PPI group for automatically dissabling the ppi after a successful spi capture
#define HASH_TABLE_BITS                        11                                                             ///< How many bits will be used for the hashtable for the _end_buffers
#define HASH_TABLE_SIZE                        (1 << HASH_TABLE_BITS)                                         ///< How big will the hashtable for the _end_buffers
#define HASH_TABLE_MASK                        ((1 << HASH_TABLE_BITS) - 1)                                   ///< Mask selecting the HAS_TABLE_BITS least significant bits
#define NUM_LSFR_COUNT_CHECKPOINTS             64                                                             ///< How many lsfr checkpoints are per polynomial
#define DISTANCE_BETWEEN_LSFR_CHECKPOINTS      2048                                                           ///< How many lsfr checkpoints are per polynomial
#define CHECKPOINT_TABLE_BITS                  6                                                              ///< How many bits will be used for the checkpoint table for the lfsr search
#define CHECKPOINT_TABLE_MASK_LOW              ((1 << CHECKPOINT_TABLE_BITS) - 1)                             ///< How big will the checkpoint table for the lfsr search
#define CHECKPOINT_TABLE_MASK_HIGH             (((1 << CHECKPOINT_TABLE_BITS) - 1) << CHECKPOINT_TABLE_BITS)  ///< Mask selecting the CHECKPOINT_TABLE_BITS least significant bits
#define LH2_MAX_DATA_VALID_TIME_US             2000000                                                        //< Data older than this is considered outdate and should be erased (in microseconds)
#define LH2_SWEEP_PERIOD_US                    20000                                                          ///< time, in microseconds, between two full rotations of the LH2 motor
#define LH2_SWEEP_PERIOD_THRESHOLD_US          1000                                                           ///< How close a LH2 pulse must arrive relative to LH2_SWEEP_PERIOD_US, to be considered the same type of sweep (first sweep or second second). (in microseconds)
#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#define LH2_TIMER_DEV 2  ///< Timer device used for LH2
#else
#define LH2_TIMER_DEV 3  ///< Timer device used for LH2
#endif

#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)
#if defined(NRF_TRUSTZONE_NONSECURE)
#define NRF_SPIM NRF_SPIM4_NS
#define NRF_PPI  NRF_DPPIC_NS
#else
#define NRF_SPIM   NRF_SPIM4_S
#define NRF_GPIOTE NRF_GPIOTE0_S
#define NRF_PPI    NRF_DPPIC_S
#endif
#define SPIM_IRQ         SPIM4_IRQn
#define SPIM_IRQ_HANDLER SPIM4_IRQHandler
#else
#define NRF_SPIM         NRF_SPIM3
#define SPIM_IRQ         SPIM3_IRQn
#define SPIM_IRQ_HANDLER SPIM3_IRQHandler
#endif

typedef struct {
    uint8_t  buffer[LH2_BUFFER_SIZE][SPI_BUFFER_SIZE];  ///< arrays of bits for local storage, contents of SPI transfer are copied into this
    uint32_t timestamps[LH2_BUFFER_SIZE];               ///< arrays of timestamps of when different SPI transfers happened
    uint8_t  write_index;                               // Index for next write
    uint8_t  read_index;                                // Index for next read
    uint8_t  count;                                     // Number of arrays in buffer
} lh2_ring_buffer_t;

///< List of rotational periods (in microseconds) of the lighthouse basestation in all its 16 modes.
static const uint16_t _lh2_sweep_period_us[LH2_BASESTATION_COUNT] = {
    19979,
    19938,
    19854,
    19771,
    19729,
    19646,
    19604,
    19563,
    19521,
    19354,
    19146,
    18979,
    18896,
    18771,
    18604,
    18479,
};

static const uint32_t _periods[LH2_BASESTATION_COUNT] = {
    959000,
    957000,
    953000,
    949000,
    947000,
    943000,
    941000,
    939000,
    937000,
    929000,
    919000,
    911000,
    907000,
    901900,
    893000,
    887000,
};

typedef struct {
    uint8_t            spi_rx_buffer[SPI_BUFFER_SIZE];  ///< buffer where data coming from SPI are stored
    lh2_ring_buffer_t  data;                            ///< array containing demodulation data of each locations
    _lfsr_checkpoint_t checkpoint;                      ///< Dynamic checkpoints for the lsfr index search
} lh2_vars_t;

//=========================== variables ========================================

static uint16_t _end_buffers_hashtable[HASH_TABLE_SIZE] = { 0 };

// Dynamic checkpoint
static uint32_t _lfsr_checkpoint_bits[LH2_POLYNOMIAL_COUNT][LH2_SWEEP_COUNT]  = { 0 };  ///<
static uint32_t _lfsr_checkpoint_count[LH2_POLYNOMIAL_COUNT][LH2_SWEEP_COUNT] = { 0 };
static uint32_t _lsfr_checkpoint_average                                      = 0;

///! NOTE: SPIM needs an SCK pin to be defined, P1.6 is used because it's not an available pin in the BCM module
static const gpio_t _lh2_spi_fake_sck_gpio = {
    .port = 1,
    .pin  = 3,
};

///< Encodes in two bits which sweep slot of a particular basestation is empty, (1 means data, 0 means empty)
typedef enum {
    LH2_SWEEP_BOTH_SLOTS_EMPTY,   ///< Both sweep slots are empty
    LH2_SWEEP_SECOND_SLOT_EMPTY,  ///< Only the second sweep slot is empty
    LH2_SWEEP_FIRST_SLOT_EMPTY,   ///< Only the first sweep slot is empty
    LH2_SWEEP_BOTH_SLOTS_FULL,    ///< Both sweep slots are filled with raw data
} db_lh2_sweep_slot_state_t;

static double homography_matrix[LH2_BASESTATION_COUNT][3][3] = { 0 };

static lh2_vars_t _lh2_vars;  ///< local data of the LH2 driver

//=========================== prototypes =======================================

// these functions are called in the order written to perform the LH2 localization
/**
 * @brief wiggle the data and envelope lines in a magical way to configure the TS4231 to continuously read for LH2 sweep signals.
 *
 * @param[in]   gpio_d  pointer to gpio data
 * @param[in]   gpio_e  pointer to gpio event
 * @return bool
 */
bool _initialize_ts4231(const gpio_t *gpio_d, const gpio_t *gpio_e);

/**
 * @brief finds the position of a 17-bit sequence (bits) in the sequence generated by polynomial3 with initial seed 1
 *
 * @param[in] index: index of polynomial
 * @param[in] bits: 17-bit sequence
 *
 * @return count: location of the sequence
 */
uint32_t _reverse_count_p(uint8_t index, uint32_t bits);

/**
 * @brief Set a gpio as an INPUT with no pull-up or pull-down
 * @param[in] gpio: pin to configure as input [0-31]
 */
void _lh2_pin_set_input(const gpio_t *gpio);

/**
 * @brief Set a gpio as an OUTPUT with standard drive
 * @param[in] gpio: gpio to configure as input [0-31]
 */
void _lh2_pin_set_output(const gpio_t *gpio);

/**
 * @brief set-up GPIOTE so that events are configured for falling (GPIOTE_CH_IN) and rising edges (GPIOTE_CH_IN_ENV_HiToLo) of the envelope signal
 *
 * @param[in]   gpio_e  pointer to gpio event
 */
void _gpiote_setup(const gpio_t *gpio_e);

/**
 * @brief start SPI3 at falling edge of envelope, start timer2 at falling edge of envelope, stop/capture timer2 at rising edge of envelope
 */
void _ppi_setup(void);

/**
 * @brief spi3 setup
 *
 * @param[in]   gpio_d  pointer to gpio data
 */
void _spi_setup(const gpio_t *gpio_d);

/**
 * @brief add one element to the ring buffer for spi captures
 *
 * @param[in]   cb          pointer to ring buffer structure
 * @param[in]   data        pointer to the data array to save in the ring buffer
 * @param[in]   timestamp   timestamp of when the LH2 measurement was taken. (taken with timer_hf_now())
 */
void _add_to_spi_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, uint32_t timestamp);

/**
 * @brief retreive the oldest element from the ring buffer for spi captures
 *
 * @param[in]    cb          pointer to ring buffer structure
 * @param[out]   data        pointer to the array where the ring buffer data will be saved
 * @param[out]   timestamp   timestamp of when the LH2 measurement was taken. (taken with timer_hf_now())
 */
bool _get_from_spi_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, uint32_t *timestamp);

/**
 * @brief Accesses the global tables _lfsr_checkpoint_hashtable & _lfsr_checkpoint_count
 *        and updates them with the last found polynomial count
 *
 * @param[in] polynomial: index of polynomial
 * @param[in] bits: 17-bit sequence
 * @param[in] count: position of the received laser sweep in the LSFR sequence
 * @param[in] sweep: index of the sweep (0 or 1)
 */
void _update_lfsr_checkpoints(uint8_t polynomial, uint32_t bits, uint32_t count, uint8_t sweep);

/**
 * @brief LH2 sweeps come with an almost perfect 20ms difference.
 *        this function uses the timestamps to figure to which sweep-slot the new LH2 data belongs to.
 *
 * @param[in] lh2 pointer to the lh2 instance
 * @param[in] polynomial: index of found polynomia
 * @param[in] timestamp: timestamp of the SPI capture
 *
 */
uint8_t _select_sweep(db_lh2_t *lh2, uint8_t polynomial, uint32_t timestamp);

//=========================== public ===========================================

bool db_lh2_init(db_lh2_t *lh2, const gpio_t *gpio_d, const gpio_t *gpio_e) {

#if defined(BOARD_DOTBOT_V3)
    // DotBot-v3 has its own LH enable pin
    gpio_t lh_en = { .port = 0, .pin = 16 };
    db_gpio_init(&lh_en, DB_GPIO_OUT);
    db_gpio_set(&lh_en);
#endif
    // Initialize the TS4231 on power-up - this is only necessary when power-cycling
    if (!_initialize_ts4231(gpio_d, gpio_e)) {
        // TS4231 initialization failed
        return false;
    }

    // Configure the necessary Pins in the GPIO peripheral  (MOSI and CS not needed)
    _lh2_pin_set_input(gpio_d);                    // Data_pin will become the MISO pin
    _lh2_pin_set_output(&_lh2_spi_fake_sck_gpio);  // set SCK as Output.

    _spi_setup(gpio_d);

    // Setup the LH2 local variables
    memset(_lh2_vars.spi_rx_buffer, 0, SPI_BUFFER_SIZE);
    // initialize the spi ring buffer
    memset(&_lh2_vars.data, 0, sizeof(lh2_ring_buffer_t));

    // Setup LH2 data
    lh2->spi_ring_buffer_count_ptr = &_lh2_vars.data.count;  // pointer to the size of the spi ring buffer,

    for (uint8_t sweep = 0; sweep < LH2_SWEEP_COUNT; sweep++) {
        for (uint8_t basestation = 0; basestation < LH2_BASESTATION_COUNT; basestation++) {
            lh2->locations[sweep][basestation].lfsr_counts         = LH2_LOCATION_ERROR_INDICATOR;
            lh2->locations[sweep][basestation].selected_polynomial = LH2_POLYNOMIAL_ERROR_INDICATOR;
            lh2->timestamps[sweep][basestation]                    = 0;
            lh2->data_ready[sweep][basestation]                    = DB_LH2_NO_NEW_DATA;
        }
    }
    memset(_lh2_vars.data.buffer[0], 0, LH2_BUFFER_SIZE);

    // initialize GPIOTEs
    _gpiote_setup(gpio_e);

    // initialize PPI
    _ppi_setup();

    return true;
}

void db_lh2_start(void) {

    NRF_PPI->TASKS_CHG[PPI_SPI_GROUP].EN = 1;
}

void db_lh2_stop(void) {

    NRF_PPI->TASKS_CHG[PPI_SPI_GROUP].DIS = 1;
}

void db_lh2_reset(db_lh2_t *lh2) {
    // compute LFSR locations and detect invalid packets
    for (uint8_t basestation = 0; basestation < LH2_BASESTATION_COUNT; basestation++) {
        for (uint8_t sweep = 0; sweep < 2; sweep++) {

            // Remove the flags indicating available data
            lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
            lh2->timestamps[sweep][basestation] = 0;
            // We won't actually clear the data, it's not worth the computational effort.
        }
    }
}

void db_lh2_process_location(db_lh2_t *lh2) {
    // There is no TS4231 data to process, return early.
    if (_lh2_vars.data.count == 0) {
        return;
    }

    //*********************************************************************************//
    //                              Prepare Raw Data                                   //
    //*********************************************************************************//

    // Get value before it's overwritten by the ringbuffer.
    uint8_t temp_spi_bits[SPI_BUFFER_SIZE * 2] = { 0 };  // The temp buffer has to be 128 long because _demodulate_light() expects it to be so
                                                         // Making it smaller causes a hardfault
                                                         // I don't know why, the SPI buffer is clearly 64bytes long.
                                                         // should ask fil about this

    // stop the interruptions while you're reading the data.
    uint32_t temp_timestamp = 0;  // default timestamp
    if (!_get_from_spi_ring_buffer(&_lh2_vars.data, temp_spi_bits, &temp_timestamp)) {
        return;
    }

// Check if Qualysis Mocap data is interfering with the SPI capture
#if defined(LH2_MOCAP_FILTER)
    if (_check_mocap_interference(temp_spi_bits, SPI_BUFFER_SIZE)) {
        return;  // if a qualysis pulse caused a false spi trigger, leave the function.
    }
#endif

    // perform the demodulation received packets
    // convert the SPI reading to bits via zero-crossing counter demodulation and differential/biphasic manchester decoding.
    uint64_t temp_bits_sweep = _demodulate_light(temp_spi_bits);

    // figure out which polynomial the data belongs  to
    int8_t  temp_bit_offset          = 0;  // default offset
    uint8_t temp_selected_polynomial = _determine_polynomial(temp_bits_sweep, &temp_bit_offset);

    // If there was an error with the polynomial, leave without updating anything
    if (temp_selected_polynomial == LH2_POLYNOMIAL_ERROR_INDICATOR) {
        return;
    }

    // Figure out in which of the two sweep slots we should save the new data.
    uint8_t sweep = _select_sweep(lh2, temp_selected_polynomial, temp_timestamp);

    // Compute which basestation the sweep came from (polynomial 0,1 must map to LH0, 2,3 to LH1, etc... This can be accomplish by  integer-dividing the selected poly in 2, a shift >> accomplishes this.)
    uint8_t basestation = temp_selected_polynomial >> 1;

    //*********************************************************************************//
    //                             Compute LFSR Position                               //
    //*********************************************************************************//

    // Select the valid bits of the lfsr by applying the offset (he first few bits might be invalid, as detected by _determine_polynomial())
    uint32_t temp_lfsr_bits = temp_bits_sweep >> (47 - temp_bit_offset);

    // Sanity check, make sure you don't start the LFSR search with a bit-sequence full of zeros.
    if (temp_lfsr_bits == 0x000000) {
        // Mark the data as wrong and keep going
        lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
        return;
    }

    // Compute the lfsr location.
    uint32_t temp_lfsr_loc = _lfsr_index_search(&_lh2_vars.checkpoint,
                                                temp_selected_polynomial,
                                                temp_lfsr_bits);

    // Check that the count didn't fall on an illegal value
    if (temp_lfsr_loc != LH2_LFSR_SEARCH_ERROR_INDICATOR) {
        // Save a new dynamic checkpoint
        _update_lfsr_checkpoints(temp_selected_polynomial, temp_lfsr_bits, temp_lfsr_loc, sweep);
    } else {
        // Mark the data as wrong and keep going
        lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
        return;
    }

    // Undo the bit offset introduced above, to get the LFSR position of the first bit that hit the sensor.
    temp_lfsr_loc -= temp_bit_offset;

    //*********************************************************************************//
    //                                 Store results                                   //
    //*********************************************************************************//

    // Save raw data information
    lh2->timestamps[sweep][basestation] = temp_timestamp;
    // Save processed location information
    lh2->locations[sweep][basestation].lfsr_counts         = temp_lfsr_loc;
    lh2->locations[sweep][basestation].selected_polynomial = temp_selected_polynomial;
    // Mark the data point as processed
    lh2->data_ready[sweep][basestation] = DB_LH2_PROCESSED_DATA_AVAILABLE;
}

void db_lh2_calculate_position(uint32_t count1, uint32_t count2, uint32_t basestation_index, double *coordinates) {

    double alpha_1 = ((double)(count1) * 8.0 / _periods[basestation_index]) * 2.0 * M_PI;
    double alpha_2 = ((double)(count2) * 8.0 / _periods[basestation_index]) * 2.0 * M_PI;

    double cam_x = -tan(0.5 * (alpha_1 + alpha_2));
    double cam_y = 0;

    if (count1 < count2) {
        cam_y = -sin(alpha_2 / 2 - alpha_1 / 2 - 60 * M_PI / 180) / tan(M_PI / 6);
    } else {
        cam_y = -sin(alpha_1 / 2 - alpha_2 / 2 - 60 * M_PI / 180) / tan(M_PI / 6);
    };

    double x_position = homography_matrix[basestation_index][0][0] * cam_x + homography_matrix[basestation_index][0][1] * cam_y + homography_matrix[basestation_index][0][2];
    double y_position = homography_matrix[basestation_index][1][0] * cam_x + homography_matrix[basestation_index][1][1] * cam_y + homography_matrix[basestation_index][1][2];
    double scale      = homography_matrix[basestation_index][2][0] * cam_x + homography_matrix[basestation_index][2][1] * cam_y + homography_matrix[basestation_index][2][2];

    coordinates[0] = x_position / scale;
    coordinates[1] = y_position / scale;
}

void db_lh2_store_homography(db_lh2_t *lh2, uint8_t basestation_index, int32_t homography_matrix_from_packet[3][3]) {
    double homography_matrix_temp_storage[3][3] = { 0 };
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            homography_matrix_temp_storage[i][j] = (double)(homography_matrix_from_packet[i][j] / 1e3);
        }
    }
    memcpy(homography_matrix[basestation_index], homography_matrix_temp_storage, sizeof(double) * 3 * 3);

    lh2->lh2_calibration_complete[basestation_index] = true;
}

#define TS4231_INIT_SAMPLES_LEN 14
const uint8_t expected_init_sequence[TS4231_INIT_SAMPLES_LEN] = {
    0x01, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01
};

//=========================== private ==========================================

bool _initialize_ts4231(const gpio_t *gpio_d, const gpio_t *gpio_e) {

    // Configure the wait timer
    db_timer_hf_init(LH2_TIMER_DEV);

    // Filip's code define these pins as inputs, and then changes them quickly to outputs. Not sure why, but it works.
    _lh2_pin_set_input(gpio_d);
    _lh2_pin_set_input(gpio_e);

    // start the TS4231 initialization
    // Wiggle the Envelope and Data pins
    _lh2_pin_set_output(gpio_e);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;  // set pin HIGH
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;  // set pin LOW
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    _lh2_pin_set_output(gpio_d);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTSET = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    // Turn the pins back to inputs
    _lh2_pin_set_input(gpio_d);
    _lh2_pin_set_input(gpio_e);
    // finally, wait 1 milisecond
    db_timer_hf_delay_us(LH2_TIMER_DEV, 1000);

    // Send the configuration magic number/sequence
    uint16_t config_val = 0x392B;
    // Turn the Data and Envelope lines back to outputs and clear them.
    _lh2_pin_set_output(gpio_e);
    _lh2_pin_set_output(gpio_d);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTCLR = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    // Send the magic configuration value, MSB first.
    for (uint8_t i = 0; i < 15; i++) {

        config_val = config_val << 1;
        if ((config_val & 0x8000) > 0) {
            nrf_port[gpio_d->port]->OUTSET = 1 << gpio_d->pin;
        } else {
            nrf_port[gpio_d->port]->OUTCLR = 1 << gpio_d->pin;
        }

        // Toggle the Envelope line as a clock.
        db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
        nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
        db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
        nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
        db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    }
    // Finish send sequence and turn pins into inputs again.
    nrf_port[gpio_d->port]->OUTCLR = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTSET = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    _lh2_pin_set_input(gpio_d);
    _lh2_pin_set_input(gpio_e);
    // Finish by waiting 10usec
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);

    // Now read back the sequence that the TS4231 answers.
    _lh2_pin_set_output(gpio_e);
    _lh2_pin_set_output(gpio_d);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTCLR = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTSET = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    // Set Data pin as an input, to receive the data
    _lh2_pin_set_input(gpio_d);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);

    // Use the Envelope pin to output a clock while the initialization sequence arrives.
    uint8_t init_sequence[TS4231_INIT_SAMPLES_LEN] = { 0 };
    for (uint8_t i = 0; i < TS4231_INIT_SAMPLES_LEN; i++) {
        nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
        db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
        init_sequence[i]               = db_gpio_read(gpio_d);
        nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
        db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    }

    if (memcmp(init_sequence, expected_init_sequence, TS4231_INIT_SAMPLES_LEN)) {
#if defined(DB_LED1_PIN)
        db_gpio_init(&db_led1, DB_GPIO_OUT);
        db_gpio_set(&db_led1);
#endif
        puts("\nGot invalid initialization sequence:");
        for (uint8_t i = 0; i < TS4231_INIT_SAMPLES_LEN; i++) {
            printf("0x%02x ", init_sequence[i]);
        }
        puts("");
        return false;
    }

    // Finish the configuration procedure
    _lh2_pin_set_output(gpio_d);
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTSET = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);

    nrf_port[gpio_e->port]->OUTCLR = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_d->port]->OUTCLR = 1 << gpio_d->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);
    nrf_port[gpio_e->port]->OUTSET = 1 << gpio_e->pin;
    db_timer_hf_delay_us(LH2_TIMER_DEV, 10);

    _lh2_pin_set_input(gpio_d);
    _lh2_pin_set_input(gpio_e);

    db_timer_hf_delay_us(LH2_TIMER_DEV, 50000);

    return true;
}

void _lh2_pin_set_input(const gpio_t *gpio) {
    // Configure Data pin as INPUT, with no pullup or pull down.
    nrf_port[gpio->port]->PIN_CNF[gpio->pin] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                               (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                               (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos);
}

void _lh2_pin_set_output(const gpio_t *gpio) {
    // Configure Data pin as OUTPUT, with standar power drive current.
    nrf_port[gpio->port]->PIN_CNF[gpio->pin] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |   // Set Pin as output
                                               (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos);  // Activate high current gpio mode.
}

void _gpiote_setup(const gpio_t *gpio_e) {
    NRF_GPIOTE->CONFIG[GPIOTE_CH_IN_ENV_HiToLo] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                                  (gpio_e->pin << GPIOTE_CONFIG_PSEL_Pos) |
                                                  (gpio_e->port << GPIOTE_CONFIG_PORT_Pos) |
                                                  (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

    NRF_GPIOTE->CONFIG[GPIOTE_CH_IN_ENV_LoToHi] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                                                  (gpio_e->pin << GPIOTE_CONFIG_PSEL_Pos) |
                                                  (gpio_e->port << GPIOTE_CONFIG_PORT_Pos) |
                                                  (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);
}

void _ppi_setup(void) {
#if defined(NRF5340_XXAA) && defined(NRF_APPLICATION)

    // Add the selected DPPI Channel to a Group to be able to disable it with a Task.
    NRF_PPI->CHG[PPI_SPI_GROUP] = (1 << PPI_SPI_START_CHAN);

    // Publish an Event when the Envelope line goes from HIGH to LOW.
    NRF_GPIOTE->PUBLISH_IN[GPIOTE_CH_IN_ENV_HiToLo] = (GPIOTE_PUBLISH_IN_EN_Enabled << GPIOTE_PUBLISH_IN_EN_Pos) |
                                                      (PPI_SPI_START_CHAN << GPIOTE_PUBLISH_IN_CHIDX_Pos);

    // Subscription to trigger the SPI transfer and disable the PPI system
    NRF_SPIM->SUBSCRIBE_START = (SPIM_SUBSCRIBE_START_EN_Enabled << SPIM_SUBSCRIBE_START_EN_Pos) |
                                (PPI_SPI_START_CHAN << SPIM_SUBSCRIBE_START_CHIDX_Pos);

    NRF_PPI->SUBSCRIBE_CHG[PPI_SPI_GROUP].DIS = (DPPIC_SUBSCRIBE_CHG_DIS_EN_Enabled << DPPIC_SUBSCRIBE_CHG_DIS_EN_Pos) |
                                                (PPI_SPI_START_CHAN << DPPIC_SUBSCRIBE_CHG_DIS_CHIDX_Pos);

#else

    // Add all the ppi setup to group 0 to be able to enable and disable it automatically.
    NRF_PPI->CHG[PPI_SPI_GROUP] = (1 << PPI_SPI_START_CHAN);

    uint32_t envelope_input_HiToLo        = (uint32_t)&NRF_GPIOTE->EVENTS_IN[GPIOTE_CH_IN_ENV_HiToLo];
    uint32_t spi_start_task_addr          = (uint32_t)&NRF_SPIM->TASKS_START;
    uint32_t ppi_group0_disable_task_addr = (uint32_t)&NRF_PPI->TASKS_CHG[0].DIS;

    NRF_PPI->CH[PPI_SPI_START_CHAN].EEP   = envelope_input_HiToLo;         // envelope down
    NRF_PPI->CH[PPI_SPI_START_CHAN].TEP   = spi_start_task_addr;           // start spi3 transfer
    NRF_PPI->FORK[PPI_SPI_START_CHAN].TEP = ppi_group0_disable_task_addr;  // Disable the PPI group

#endif
}

void _spi_setup(const gpio_t *gpio_d) {
    // Define the necessary Pins in the SPIM peripheral
    NRF_SPIM->PSEL.MISO = gpio_d->pin << SPIM_PSEL_MISO_PIN_Pos |                          // Define pin number for MISO pin
                          gpio_d->port << SPIM_PSEL_MISO_PORT_Pos |                        // Define pin port for MISO pin
                          SPIM_PSEL_MISO_CONNECT_Connected << SPIM_PSEL_MISO_CONNECT_Pos;  // Enable the MISO pin

    NRF_SPIM->PSEL.SCK = _lh2_spi_fake_sck_gpio.pin << SPIM_PSEL_SCK_PIN_Pos |          // Define pin number for SCK pin
                         _lh2_spi_fake_sck_gpio.port << SPIM_PSEL_SCK_PORT_Pos |        // Define pin port for SCK pin
                         SPIM_PSEL_SCK_CONNECT_Connected << SPIM_PSEL_SCK_CONNECT_Pos;  // Enable the SCK pin

    NRF_SPIM->PSEL.MOSI = (4UL) << SPIM_PSEL_MOSI_PIN_Pos |
                          1 << SPIM_PSEL_MOSI_PORT_Pos |
                          SPIM_PSEL_MOSI_CONNECT_Connected << SPIM_PSEL_MOSI_CONNECT_Pos;

    // Configure the Interruptions
    NVIC_ClearPendingIRQ(SPIM_IRQ);
    NVIC_DisableIRQ(SPIM_IRQ);  // Disable interruptions while configuring

    // Configure the SPIM peripheral
    NRF_SPIM->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M32;                         // Set SPI frequency to 32MHz
    NRF_SPIM->CONFIG    = SPIM_CONFIG_ORDER_MsbFirst << SPIM_CONFIG_ORDER_Pos;  // Set MsB out first

    // Configure the EasyDMA channel, only using RX
    NRF_SPIM->RXD.MAXCNT = SPI_BUFFER_SIZE;                    // Set the size of the input buffer.
    NRF_SPIM->RXD.PTR    = (uint32_t)_lh2_vars.spi_rx_buffer;  // Set the input buffer pointer.

    NRF_SPIM->INTENSET = SPIM_INTENSET_END_Enabled << SPIM_INTENSET_END_Pos;  // Enable interruption for when a packet arrives
    NVIC_SetPriority(SPIM_IRQ, SPIM_INTERRUPT_PRIORITY);                      // Set priority for Radio interrupts to 1
    // Enable SPIM interruptions
    NVIC_EnableIRQ(SPIM_IRQ);

    // Enable the SPIM peripheral
    NRF_SPIM->ENABLE = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos;
}

void _add_to_spi_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, uint32_t timestamp) {

    memcpy(cb->buffer[cb->write_index], data, SPI_BUFFER_SIZE);
    cb->timestamps[cb->write_index] = timestamp;
    cb->write_index                 = (cb->write_index + 1) % LH2_BUFFER_SIZE;

    if (cb->count < LH2_BUFFER_SIZE) {
        cb->count++;
    } else {
        // Overwrite oldest data, adjust read_index
        cb->read_index = (cb->read_index + 1) % LH2_BUFFER_SIZE;
    }
}

bool _get_from_spi_ring_buffer(lh2_ring_buffer_t *cb, uint8_t *data, uint32_t *timestamp) {
    if (cb->count == 0) {
        // Buffer is empty
        return false;
    }

    memcpy(data, cb->buffer[cb->read_index], SPI_BUFFER_SIZE);
    *timestamp     = cb->timestamps[cb->read_index];
    cb->read_index = (cb->read_index + 1) % LH2_BUFFER_SIZE;
    cb->count--;

    return true;
}
void _update_lfsr_checkpoints(uint8_t polynomial, uint32_t bits, uint32_t count, uint8_t sweep) {

    // Save the new count in the correct place in the checkpoint array
    _lh2_vars.checkpoint.bits[polynomial][sweep]  = bits;
    _lh2_vars.checkpoint.count[polynomial][sweep] = count;
}

uint8_t _select_sweep(db_lh2_t *lh2, uint8_t polynomial, uint32_t timestamp) {
    // TODO: check the exact, per-mode period of each polynomial instead of using a blanket 20ms

    uint8_t  basestation  = polynomial >> 1;  ///< each base station uses 2 polynomials. integer dividing by 2 maps the polynomial number to the basestation number.
    uint16_t sweep_period = _lh2_sweep_period_us[basestation];
    uint32_t now          = db_timer_hf_now(LH2_TIMER_DEV);

    for (size_t sweep = 0; sweep < 2; sweep++) {
        if (now - lh2->timestamps[0][basestation] > LH2_MAX_DATA_VALID_TIME_US) {
            // Remove data that is too old.
            lh2->timestamps[sweep][basestation] = 0;
            lh2->data_ready[sweep][basestation] = DB_LH2_NO_NEW_DATA;
            // I don't think it's worth it to remove the location data. It is already marked as "No new data"
        }
    }

    ///< Encode in two bits which sweep slot of this basestation is empty, (1 means data, 0 means empty)
    uint8_t sweep_slot_state = ((lh2->timestamps[1][basestation] != 0) << 1) | (lh2->timestamps[0][basestation] != 0);
    // by default, select the first slot
    uint8_t selected_sweep = 0;

    switch (sweep_slot_state) {

        case LH2_SWEEP_BOTH_SLOTS_EMPTY:
        {
            // use the first slot
            selected_sweep = 0;
            break;
        }

        case LH2_SWEEP_FIRST_SLOT_EMPTY:
        {
            // check that the filled slot is not a perfect 20ms match to the new data.
            uint32_t diff = (timestamp - lh2->timestamps[1][basestation]) % sweep_period;
            diff          = diff < sweep_period - diff ? diff : sweep_period - diff;

            if (diff < LH2_SWEEP_PERIOD_THRESHOLD_US) {
                // match: use filled slot
                selected_sweep = 1;
            } else {
                // no match: use empty slot
                selected_sweep = 0;
            }
            break;
        }

        case LH2_SWEEP_SECOND_SLOT_EMPTY:
        {
            // check that the filled slot is not a perfect 20ms match to the new data.
            uint32_t diff = (timestamp - lh2->timestamps[0][basestation]) % sweep_period;
            diff          = diff < sweep_period - diff ? diff : sweep_period - diff;

            if (diff < LH2_SWEEP_PERIOD_THRESHOLD_US) {
                // match: use filled slot
                selected_sweep = 0;
            } else {
                // no match: use empty slot
                selected_sweep = 1;
            }
            break;
        }

        case LH2_SWEEP_BOTH_SLOTS_FULL:
        {
            // How far away is this new pulse from the already stored data
            int64_t diff_0 = ((timestamp - lh2->timestamps[0][basestation]) % sweep_period);
            diff_0         = diff_0 < sweep_period - diff_0 ? diff_0 : sweep_period - diff_0;
            int64_t diff_1 = ((timestamp - lh2->timestamps[1][basestation]) % sweep_period);
            diff_1         = diff_1 < sweep_period - diff_1 ? diff_1 : sweep_period - diff_1;

            // Use the one that is closest to 20ms
            if (diff_0 <= diff_1) {
                selected_sweep = 0;
            } else {
                selected_sweep = 1;
            }
            break;
        }

        default:
        {
            // By default, use he first slot
            selected_sweep = 0;
            break;
        }
    }

    return selected_sweep;
}

void db_lh2_handle_isr(void) {
    // Reenable the PPI channel
    db_lh2_start();
    // Read the current time.
    uint32_t timestamp = db_timer_hf_now(LH2_TIMER_DEV);
    // Add new reading to the ring buffer
    _add_to_spi_ring_buffer(&_lh2_vars.data, _lh2_vars.spi_rx_buffer, timestamp);
}

//=========================== interrupts =======================================

void SPIM_IRQ_HANDLER(void) {
    // Check if the interrupt was caused by a fully send package
    if (NRF_SPIM->EVENTS_END) {
        // Clear the Interrupt flag
        NRF_SPIM->EVENTS_END = 0;
        db_lh2_handle_isr();
    }
}
