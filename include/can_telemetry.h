/**
 * @file can_telemetry.h
 * @brief EV Powertrain & Battery Telemetry Encoding, Decoding, and Safety Supervision.
 * 
 * Target Message: ID 0x120 (Cycle: 100ms, Standard 11-bit ID)
 * Features:
 *  - Deterministic 8-byte bitfield packing & unpacking.
 *  - Fixed-point scaling (no expensive float operations in interrupt context).
 *  - Over-temperature and voltage cutoff safety threshold checking.
 *  - 300ms heartbeat loss timeout detection.
 * 
 * @author S. Sherifred Singh
 */

#ifndef CAN_TELEMETRY_H
#define CAN_TELEMETRY_H

#include "can_driver.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Target Message Configuration */
#define TELEMETRY_CAN_ID            (0x120U)
#define TELEMETRY_CYCLE_TIME_MS     (100U)
#define TELEMETRY_TIMEOUT_LIMIT_MS  (300U)

/* Safety Threshold Limits */
#define BATT_OVER_TEMP_THRESHOLD_C  (60)     /* 60 degC maximum allowable cell temp */
#define BATT_UNDER_VOLT_CUTOFF_MV   (42000U) /* 42.0V minimum pack cutoff */
#define BATT_OVER_VOLT_LIMIT_MV     (58400U) /* 58.4V maximum pack charge limit */

/* Safety Fault Bitflags */
#define FAULT_OVER_TEMP             (1U << 0)
#define FAULT_UNDER_VOLTAGE         (1U << 1)
#define FAULT_OVER_VOLTAGE          (1U << 2)
#define FAULT_COMM_TIMEOUT          (1U << 3)
#define FAULT_COUNTER_FROZEN        (1U << 4)

/* EV Powertrain Telemetry Data Structure */
typedef struct {
    uint16_t pack_voltage_mv;   /* Battery Pack Voltage in millivolts (e.g., 52400 = 52.4V) */
    uint16_t motor_rpm;         /* Motor Speed in RPM (0 to 8000) */
    int8_t   pack_temp_c;       /* Cell temperature in degC (-20 to +100) */
    uint8_t  vehicle_speed_kmh; /* Vehicle Speed in km/h (0 to 180) */
    uint8_t  status_flags;      /* Bit 0: Inverter Ready, Bit 1: Brake Active, Bit 2: Charging */
    uint8_t  rolling_counter;   /* 4-bit sequence alive counter (0..15) */
    uint32_t active_faults;     /* Safety fault bitmask */
} ev_telemetry_t;

/* ========================================================================== */
/* Function Prototypes                                                        */
/* ========================================================================== */

/**
 * @brief Pack structured engineering values into an 8-byte CAN frame.
 * Uses big-endian / network byte order for multi-byte telemetry fields.
 */
void telemetry_pack(const ev_telemetry_t *data, can_msg_t *msg);

/**
 * @brief Unpack an 8-byte CAN payload back into engineering units.
 */
bool telemetry_unpack(const can_msg_t *msg, ev_telemetry_t *data);

/**
 * @brief Evaluate safety thresholds (over-temp, under-voltage, cutoff).
 * @return Bitmask of active faults (0 = All Normal).
 */
uint32_t telemetry_evaluate_safety(ev_telemetry_t *data);

/**
 * @brief Check if heartbeat has timed out based on timestamp delta.
 */
bool telemetry_is_timed_out(uint32_t last_rx_ms, uint32_t current_ms);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TELEMETRY_H */
