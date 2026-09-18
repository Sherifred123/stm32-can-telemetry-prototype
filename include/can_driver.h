/**
 * @file can_driver.h
 * @brief STM32 bxCAN Peripheral Driver Interface (Loopback & Normal Modes).
 * 
 * Target: STM32F103C8T6 (ARM Cortex-M3) / Native Simulation
 * Peripheral: bxCAN (Basic Extended Controller Area Network)
 * APB1 Bus Clock: 36 MHz
 * Target Baud: 500 kbps (Nominal Automotive Rate)
 * 
 * @author S. Sherifred Singh
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* Hardware Constants & Limits                                                */
/* ========================================================================== */
#define CAN_MAX_DATA_LEN          (8U)
#define CAN_STD_ID_MAX            (0x7FFU)        /* 11-bit standard ID */
#define CAN_EXT_ID_MAX            (0x1FFFFFFFU)   /* 29-bit extended ID */

/* Operating Modes */
typedef enum {
    CAN_MODE_NORMAL   = 0x00, /* Real physical bus transmission via transceiver */
    CAN_MODE_LOOPBACK = 0x01, /* Internal self-test: TX internally routed to RX */
    CAN_MODE_SILENT   = 0x02  /* Listen-only / bus monitor mode */
} can_op_mode_t;

/* Status Codes */
typedef enum {
    CAN_OK              = 0,
    CAN_ERR_PARAM       = -1,
    CAN_ERR_TIMEOUT     = -2,
    CAN_ERR_TX_FULL     = -3,
    CAN_ERR_RX_EMPTY    = -4,
    CAN_ERR_BUS_OFF     = -5,
    CAN_ERR_FILTER_FULL = -6
} can_status_t;

/* Standard CAN 2.0B Frame Structure */
typedef struct {
    uint32_t id;                       /* 11-bit Standard or 29-bit Extended ID */
    bool     is_extended;              /* false = 11-bit Std, true = 29-bit Ext */
    bool     is_rtr;                   /* Remote Transmission Request flag */
    uint8_t  dlc;                      /* Data length (0 to 8 bytes) */
    uint8_t  data[CAN_MAX_DATA_LEN];   /* Payload data */
    uint32_t timestamp_ms;             /* Reception timestamp in milliseconds */
} can_msg_t;

/* Bit Timing Configuration (ISO 11898-1) */
typedef struct {
    uint32_t apb1_clock_hz; /* Peripheral clock (36 MHz for STM32F103) */
    uint32_t baudrate;      /* Target baud rate (500000 bps) */
    uint16_t brp;           /* Baud Rate Prescaler */
    uint8_t  ts1;           /* Time Segment 1 (Prop_Seg + Phase_Seg1) in Tq */
    uint8_t  ts2;           /* Time Segment 2 (Phase_Seg2) in Tq */
    uint8_t  sjw;           /* Resynchronization Jump Width in Tq */
    float    sample_point;  /* Resulting sample point percentage (e.g., 83.3%) */
} can_bit_timing_t;

/* Filter Bank Configuration (32-bit Identifier Mask Mode) */
typedef struct {
    uint8_t  bank_id;       /* Filter Bank index (0 to 13 on single-CAN MCU) */
    uint32_t filter_id;     /* Standard 11-bit ID to match */
    uint32_t filter_mask;   /* 11-bit mask (1 = must match, 0 = don't care) */
    uint8_t  fifo_assign;   /* Target FIFO (0 or 1) */
    bool     enabled;       /* Enable/disable state */
} can_filter_t;

/* Driver Runtime Statistics */
typedef struct {
    uint32_t tx_success_count;
    uint32_t tx_fail_count;
    uint32_t rx_count;
    uint32_t rx_filtered_count; /* Frames accepted by hardware filter */
    uint32_t rx_dropped_count;  /* Frames rejected by hardware filter */
    uint8_t  tec;               /* Transmit Error Counter */
    uint8_t  rec;               /* Receive Error Counter */
} can_stats_t;

/* ========================================================================== */
/* Function Prototypes                                                        */
/* ========================================================================== */

/**
 * @brief Initialize the CAN controller in specified mode (Loopback / Normal).
 * Configures bit-timing registers for 500 kbps @ 36 MHz APB1.
 */
can_status_t can_driver_init(can_op_mode_t mode);

/**
 * @brief Configure a 32-bit hardware acceptance filter bank.
 * Configures the MCU's silicon filter registers to reject unlisted IDs without CPU load.
 */
can_status_t can_driver_set_filter(const can_filter_t *filter);

/**
 * @brief Transmit a standard CAN frame via hardware mailbox.
 */
can_status_t can_driver_transmit(const can_msg_t *msg);

/**
 * @brief Read a frame from the receive FIFO.
 */
can_status_t can_driver_receive(can_msg_t *msg);

/**
 * @brief Check if a new message is waiting in the RX FIFO.
 */
bool can_driver_is_rx_pending(void);

/**
 * @brief Get active driver statistics.
 */
void can_driver_get_stats(can_stats_t *stats);

/**
 * @brief Get precalculated bit timing parameters for verification.
 */
void can_driver_get_bit_timing(can_bit_timing_t *timing);

#ifdef __cplusplus
}
#endif

#endif /* CAN_DRIVER_H */
