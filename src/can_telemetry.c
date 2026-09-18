/**
 * @file can_telemetry.c
 * @brief EV Powertrain & Battery Telemetry Packing, Unpacking, and Safety Logic.
 * @author S. Sherifred Singh
 */

#include "can_telemetry.h"
#include <string.h>

void telemetry_pack(const ev_telemetry_t *data, can_msg_t *msg) {
    if (data == NULL || msg == NULL) {
        return;
    }

    msg->id          = TELEMETRY_CAN_ID;
    msg->is_extended = false;
    msg->is_rtr      = false;
    msg->dlc         = CAN_MAX_DATA_LEN;

    /* Big-endian packing for multi-byte values */
    msg->data[0] = (uint8_t)((data->pack_voltage_mv >> 8) & 0xFF);
    msg->data[1] = (uint8_t)(data->pack_voltage_mv & 0xFF);

    msg->data[2] = (uint8_t)((data->motor_rpm >> 8) & 0xFF);
    msg->data[3] = (uint8_t)(data->motor_rpm & 0xFF);

    msg->data[4] = (uint8_t)data->pack_temp_c;
    msg->data[5] = data->vehicle_speed_kmh;
    msg->data[6] = data->status_flags;
    msg->data[7] = (uint8_t)(data->rolling_counter & 0x0F);
}

bool telemetry_unpack(const can_msg_t *msg, ev_telemetry_t *data) {
    if (msg == NULL || data == NULL) {
        return false;
    }

    if (msg->id != TELEMETRY_CAN_ID || msg->dlc != CAN_MAX_DATA_LEN) {
        return false;
    }

    data->pack_voltage_mv   = (uint16_t)(((uint16_t)msg->data[0] << 8) | msg->data[1]);
    data->motor_rpm         = (uint16_t)(((uint16_t)msg->data[2] << 8) | msg->data[3]);
    data->pack_temp_c       = (int8_t)msg->data[4];
    data->vehicle_speed_kmh = msg->data[5];
    data->status_flags      = msg->data[6];
    data->rolling_counter   = (uint8_t)(msg->data[7] & 0x0F);

    /* Run immediate safety threshold evaluation */
    telemetry_evaluate_safety(data);

    return true;
}

uint32_t telemetry_evaluate_safety(ev_telemetry_t *data) {
    if (data == NULL) {
        return 0;
    }

    uint32_t faults = 0;

    /* Over-temperature alarm (thermal runaway protection) */
    if (data->pack_temp_c >= BATT_OVER_TEMP_THRESHOLD_C) {
        faults |= FAULT_OVER_TEMP;
    }

    /* Under-voltage cutoff alarm (deep discharge protection) */
    if (data->pack_voltage_mv <= BATT_UNDER_VOLT_CUTOFF_MV) {
        faults |= FAULT_UNDER_VOLTAGE;
    }

    /* Over-voltage protection */
    if (data->pack_voltage_mv >= BATT_OVER_VOLT_LIMIT_MV) {
        faults |= FAULT_OVER_VOLTAGE;
    }

    data->active_faults = faults;
    return faults;
}

bool telemetry_is_timed_out(uint32_t last_rx_ms, uint32_t current_ms) {
    if (current_ms < last_rx_ms) {
        /* Handle 32-bit timer rollover */
        return ((0xFFFFFFFFU - last_rx_ms + current_ms) >= TELEMETRY_TIMEOUT_LIMIT_MS);
    }
    return ((current_ms - last_rx_ms) >= TELEMETRY_TIMEOUT_LIMIT_MS);
}
