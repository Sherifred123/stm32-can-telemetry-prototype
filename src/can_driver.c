/**
 * @file can_driver.c
 * @brief STM32 bxCAN Controller Implementation with Hardware Filter Bank Modeling.
 * 
 * Demonstrates:
 *  - 500 kbps bit timing configuration for 36 MHz APB1 peripheral clock.
 *  - 32-bit Identifier Mask hardware acceptance filtering.
 *  - Non-blocking TX mailbox dispatching and RX FIFO queuing.
 *  - Silicon Loopback mode for single-board validation without transceivers.
 * 
 * @author S. Sherifred Singh
 */

#include "can_driver.h"
#include <string.h>

/* RX FIFO Depth (matches dual 3-stage hardware FIFOs on STM32 bxCAN) */
#define RX_FIFO_CAPACITY  (16U)
#define MAX_FILTER_BANKS  (14U)

/* Module Private State */
typedef struct {
    can_msg_t buffer[RX_FIFO_CAPACITY];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} rx_fifo_t;

static rx_fifo_t       s_rx_fifo;
static can_op_mode_t   s_current_mode = CAN_MODE_LOOPBACK;
static can_filter_t    s_filters[MAX_FILTER_BANKS];
static bool            s_is_initialized = false;
static can_stats_t     s_stats;

/* Static Precomputed Bit Timing for STM32F103 (36 MHz APB1 -> 500 kbps) */
static const can_bit_timing_t s_timing_500k = {
    .apb1_clock_hz = 36000000U,
    .baudrate      = 500000U,
    .brp           = 4U,       /* Prescaler: Tq = 4 / 36 MHz = 111.11 ns */
    .ts1           = 14U,      /* Prop_Seg + Phase_Seg1 = 14 Tq */
    .ts2           = 3U,       /* Phase_Seg2 = 3 Tq */
    .sjw           = 1U,       /* Resynchronization Jump Width = 1 Tq */
    .sample_point  = 83.33f    /* Sample point = (1 + 14) / 18 = 83.33% */
};

/* ========================================================================== */
/* Internal Helper: Hardware Filter Evaluation                                */
/* ========================================================================== */

/**
 * @brief Evaluates an incoming frame against configured 32-bit filter banks.
 * Simulates the exact hardware logic performed inside the STM32 CAN filter block.
 * 
 * In STM32 bxCAN registers (CAN_FxR1 / CAN_FxR2):
 * Standard 11-bit ID is mapped to bits [31:21].
 */
static bool evaluate_hardware_filters(const can_msg_t *msg) {
    bool has_active_filter = false;

    for (uint8_t i = 0; i < MAX_FILTER_BANKS; i++) {
        if (!s_filters[i].enabled) {
            continue;
        }

        has_active_filter = true;

        /* Mask mode comparison: (ID & MASK) == (FILTER_ID & MASK) */
        uint32_t masked_incoming = msg->id & s_filters[i].filter_mask;
        uint32_t masked_target   = s_filters[i].filter_id & s_filters[i].filter_mask;

        if (masked_incoming == masked_target) {
            return true; /* Hardware filter matched! Frame allowed into FIFO */
        }
    }

    /* If no filters are active, controller accepts all frames by default.
     * Once at least one filter is enabled, unlisted IDs are blocked. */
    return !has_active_filter;
}

/* ========================================================================== */
/* Public API Implementation                                                  */
/* ========================================================================== */

can_status_t can_driver_init(can_op_mode_t mode) {
    s_current_mode = mode;

    /* Reset FIFO pointers */
    s_rx_fifo.head  = 0;
    s_rx_fifo.tail  = 0;
    s_rx_fifo.count = 0;

    /* Reset filters and statistics */
    memset(s_filters, 0, sizeof(s_filters));
    memset(&s_stats, 0, sizeof(s_stats));

    s_is_initialized = true;
    return CAN_OK;
}

can_status_t can_driver_set_filter(const can_filter_t *filter) {
    if (filter == NULL || filter->bank_id >= MAX_FILTER_BANKS) {
        return CAN_ERR_PARAM;
    }

    s_filters[filter->bank_id] = *filter;
    return CAN_OK;
}

can_status_t can_driver_transmit(const can_msg_t *msg) {
    if (!s_is_initialized || msg == NULL) {
        return CAN_ERR_PARAM;
    }

    if (msg->dlc > CAN_MAX_DATA_LEN) {
        s_stats.tx_fail_count++;
        return CAN_ERR_PARAM;
    }

    /* On real STM32 hardware, this writes to CAN_TI0R / CAN_TDT0R / CAN_TDL0R / CAN_TDH0R
     * and sets the TXRQ bit. */

    s_stats.tx_success_count++;

    /* In Loopback Mode, the silicon transmitter routes the frame internally to RX */
    if (s_current_mode == CAN_MODE_LOOPBACK) {
        s_stats.rx_count++;

        /* Pass through hardware filter block */
        if (evaluate_hardware_filters(msg)) {
            /* Filter accepted: Enqueue into RX FIFO */
            if (s_rx_fifo.count < RX_FIFO_CAPACITY) {
                s_rx_fifo.buffer[s_rx_fifo.head] = *msg;
                s_rx_fifo.head = (uint8_t)((s_rx_fifo.head + 1) % RX_FIFO_CAPACITY);
                s_rx_fifo.count++;
                s_stats.rx_filtered_count++;
            } else {
                /* FIFO Overflow */
                s_stats.rec++;
                return CAN_ERR_RX_EMPTY;
            }
        } else {
            /* Frame silently rejected by hardware filter (zero CPU overhead) */
            s_stats.rx_dropped_count++;
        }
    }

    return CAN_OK;
}

can_status_t can_driver_receive(can_msg_t *msg) {
    if (!s_is_initialized || msg == NULL) {
        return CAN_ERR_PARAM;
    }

    if (s_rx_fifo.count == 0) {
        return CAN_ERR_RX_EMPTY;
    }

    /* Read from FIFO */
    *msg = s_rx_fifo.buffer[s_rx_fifo.tail];
    s_rx_fifo.tail = (uint8_t)((s_rx_fifo.tail + 1) % RX_FIFO_CAPACITY);
    s_rx_fifo.count--;

    return CAN_OK;
}

bool can_driver_is_rx_pending(void) {
    return (s_rx_fifo.count > 0);
}

void can_driver_get_stats(can_stats_t *stats) {
    if (stats != NULL) {
        *stats = s_stats;
    }
}

void can_driver_get_bit_timing(can_bit_timing_t *timing) {
    if (timing != NULL) {
        *timing = s_timing_500k;
    }
}
