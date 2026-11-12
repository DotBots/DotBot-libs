/**
 * @file
 * @ingroup bsp_lh2
 *
 * @brief  nRF52 and nRF5340-app capture layer of the "lh2" bsp module
 *
 * @author Filip Maksimovic <filip.maksimovic@inria.fr>
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022-present
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <nrf.h>

#include "gpio.h"
#include "lh2.h"
#include "lh2_decoder.h"
#include "timer_hf.h"
#include "board_config.h"

//=========================== defines =========================================

#define SPIM_INTERRUPT_PRIORITY 2  ///< Interrupt priority, as high as it will go
#define GPIOTE_CH_IN_ENV_HiToLo 1  ///< falling edge gpio channel
#define GPIOTE_CH_IN_ENV_LoToHi 2  ///< rising edge gpio channel
#define PPI_SPI_START_CHAN      2  ///< PPI channel for starting the GPIOTE to SPI capture ppi
#define PPI_SPI_GROUP           0  ///< PPI group for automatically dissabling the ppi after a successful spi capture
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
    uint8_t        spi_rx_buffer[DB_LH2_CAPTURE_BYTES];  ///< the SPIM's DMA buffer
    db_lh2_queue_t queue;                                ///< captures not yet popped
} lh2_vars_t;

//=========================== variables ========================================

/// Pins the SPIM drives as SCK and MOSI: it only receives, on the data line, but runs with both
static const gpio_t _lh2_spi_fake_sck_gpio  = { .port = 1, .pin = 3 };
static const gpio_t _lh2_spi_fake_mosi_gpio = { .port = 1, .pin = 4 };

static lh2_vars_t _lh2_vars;  ///< local data of the LH2 driver

#if defined(DB_LH2_CAPTURE_RTT)
#include <RTT/SEGGER_RTT.h>

/// Every capture popped is also written to this RTT up-buffer as a db_lh2_capture_record()
#define LH2_RTT_UP_BUFFER (1U)
#define LH2_RTT_BYTES     (8192U)

static uint8_t  _rtt_buffer[LH2_RTT_BYTES];
static uint32_t _rtt_sequence = 0;
static uint32_t _rtt_dropped  = 0;  ///< records the up-buffer had no room for
#endif

//=========================== prototypes =======================================

/**
 * @brief wiggle the data and envelope lines in a magical way to configure the TS4231 to continuously read for LH2 sweep signals.
 *
 * @param[in]   gpio_d  pointer to gpio data
 * @param[in]   gpio_e  pointer to gpio event
 * @return bool
 */
bool _initialize_ts4231(const gpio_t *gpio_d, const gpio_t *gpio_e);

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
 * @brief start the SPIM at the falling edge of the envelope, and disarm until re-armed
 */
void _ppi_setup(void);

/**
 * @brief SPIM setup
 *
 * @param[in]   gpio_d  pointer to gpio data
 */
void _spi_setup(const gpio_t *gpio_d);

//=========================== public ===========================================

bool db_lh2_capture_init(const gpio_t *gpio_d, const gpio_t *gpio_e) {

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

    memset(_lh2_vars.spi_rx_buffer, 0, DB_LH2_CAPTURE_BYTES);
    memset(&_lh2_vars.queue, 0, sizeof(_lh2_vars.queue));
#if defined(DB_LH2_CAPTURE_RTT)
    SEGGER_RTT_ConfigUpBuffer(LH2_RTT_UP_BUFFER, "lh2", _rtt_buffer, sizeof(_rtt_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
#endif

    // initialize GPIOTEs
    _gpiote_setup(gpio_e);

    // initialize PPI
    _ppi_setup();

    return true;
}

void db_lh2_capture_start(void) {
    NRF_PPI->TASKS_CHG[PPI_SPI_GROUP].EN = 1;
}

void db_lh2_capture_stop(void) {
    NRF_PPI->TASKS_CHG[PPI_SPI_GROUP].DIS = 1;
}

bool db_lh2_capture_pop(db_lh2_capture_t *out) {
    // The SPIM interrupt pushes: keep it out while the queue is read
    NVIC_DisableIRQ(SPIM_IRQ);
    bool popped = db_lh2_queue_pop(&_lh2_vars.queue, out);
    NVIC_EnableIRQ(SPIM_IRQ);
#if defined(DB_LH2_CAPTURE_RTT)
    if (popped) {
        uint8_t record[DB_LH2_RECORD_BYTES];
        db_lh2_capture_record(out, _rtt_sequence++, _lh2_vars.queue.dropped, _rtt_dropped, record);
        if (SEGGER_RTT_Write(LH2_RTT_UP_BUFFER, record, sizeof(record)) != sizeof(record)) {
            _rtt_dropped++;
        }
    }
#endif
    return popped;
}

void db_lh2_capture_flush(void) {
    NVIC_DisableIRQ(SPIM_IRQ);
    db_lh2_queue_clear(&_lh2_vars.queue);
    NVIC_EnableIRQ(SPIM_IRQ);
}

uint32_t db_lh2_capture_dropped(void) {
    return _lh2_vars.queue.dropped;
}

uint32_t db_lh2_capture_now_us(void) {
    return db_timer_hf_now(LH2_TIMER_DEV);
}

void db_lh2_handle_isr(void) {
    uint32_t timestamp = db_timer_hf_now(LH2_TIMER_DEV);
    // Queue the capture before re-arming: the next envelope edge restarts the
    // SPIM DMA into spi_rx_buffer
    db_lh2_queue_push(&_lh2_vars.queue, _lh2_vars.spi_rx_buffer, timestamp);
    db_lh2_capture_start();
}

//=========================== private ==========================================

#define TS4231_INIT_SAMPLES_LEN 14
static const uint8_t expected_init_sequence[TS4231_INIT_SAMPLES_LEN] = {
    0x01, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01
};

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

    NRF_SPIM->PSEL.MOSI = _lh2_spi_fake_mosi_gpio.pin << SPIM_PSEL_MOSI_PIN_Pos |
                          _lh2_spi_fake_mosi_gpio.port << SPIM_PSEL_MOSI_PORT_Pos |
                          SPIM_PSEL_MOSI_CONNECT_Connected << SPIM_PSEL_MOSI_CONNECT_Pos;

    // Configure the Interruptions
    NVIC_ClearPendingIRQ(SPIM_IRQ);
    NVIC_DisableIRQ(SPIM_IRQ);  // Disable interruptions while configuring

    // Configure the SPIM peripheral
    NRF_SPIM->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M32;                         // Set SPI frequency to 32MHz
    NRF_SPIM->CONFIG    = SPIM_CONFIG_ORDER_MsbFirst << SPIM_CONFIG_ORDER_Pos;  // Set MsB out first

    // Configure the EasyDMA channel, only using RX
    NRF_SPIM->RXD.MAXCNT = DB_LH2_CAPTURE_BYTES;               // Set the size of the input buffer.
    NRF_SPIM->RXD.PTR    = (uint32_t)_lh2_vars.spi_rx_buffer;  // Set the input buffer pointer.

    NRF_SPIM->INTENSET = SPIM_INTENSET_END_Enabled << SPIM_INTENSET_END_Pos;  // Enable interruption for when a packet arrives
    NVIC_SetPriority(SPIM_IRQ, SPIM_INTERRUPT_PRIORITY);                      // Set priority for Radio interrupts to 1
    // Enable SPIM interruptions
    NVIC_EnableIRQ(SPIM_IRQ);

    // Enable the SPIM peripheral
    NRF_SPIM->ENABLE = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos;
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
