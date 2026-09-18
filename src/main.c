/**
 * @file main.c
 * @brief STM32 CAN 2.0B Telemetry & Safety Supervision Application.
 * 
 * Demonstrates:
 *  1. bxCAN Controller Initialization at 500 kbps (36 MHz APB1).
 *  2. 32-bit Hardware Acceptance Filter Bank setup (ID 0x120, Mask 0x7FF).
 *  3. Non-blocking frame transmission and reception via Loopback Mode.
 *  4. EV Powertrain Telemetry packing & unpacking.
 *  5. Hardware filter rejection verification (Unauthorized ID rejected).
 *  6. Real-time safety alarm triggering (Over-temperature condition).
 *  7. Communication loss heartbeat timeout detection (300ms threshold).
 * 
 * @author S. Sherifred Singh (Senior Embedded Firmware Engineer)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "can_driver.h"
#include "can_telemetry.h"

/* Helper to print a clean line banner */
static void print_separator(void) {
    printf("+-------------------------------------------------------------------------+\n");
}

/* Helper to print a raw 8-byte CAN frame */
static void print_can_frame(const char *prefix, const can_msg_t *msg) {
    printf("%s [ID: 0x%03X | DLC: %d | Data:", prefix, msg->id, msg->dlc);
    for (uint8_t i = 0; i < msg->dlc; i++) {
        printf(" %02X", msg->data[i]);
    }
    printf("]\n");
}

int main(void) {
    print_separator();
    printf("|   STM32F103 bxCAN 2.0B Telemetry & Safety Node - Engineering Prototype  |\n");
    printf("|   Author: S. Sherifred Singh | Target: 500 kbps Automotive Bus         |\n");
    print_separator();

    /* ---------------------------------------------------------------------- */
    /* Step 1: Initialize CAN Controller in Silicon Loopback Mode             */
    /* ---------------------------------------------------------------------- */
    can_status_t status = can_driver_init(CAN_MODE_LOOPBACK);
    if (status != CAN_OK) {
        printf("[ERROR] CAN Driver Initialization Failed (Code: %d)\n", status);
        return -1;
    }

    can_bit_timing_t timing;
    can_driver_get_bit_timing(&timing);

    printf("[INIT] Mode: LOOPBACK (Internal Silicon Self-Test)\n");
    printf("[INIT] Clock: %u MHz | Baudrate: %u kbps | Prescaler: %d\n",
           timing.apb1_clock_hz / 1000000U, timing.baudrate / 1000U, timing.brp);
    printf("[INIT] Bit Segments: TS1=%d Tq, TS2=%d Tq, SJW=%d Tq | Sample Point: %.1f%%\n",
           timing.ts1, timing.ts2, timing.sjw, timing.sample_point);

    /* ---------------------------------------------------------------------- */
    /* Step 2: Configure 32-bit Hardware Acceptance Filter Bank 0             */
    /* ---------------------------------------------------------------------- */
    /* Target: Accept ONLY ID 0x120. All 11 bits must match (Mask: 0x7FF).   */
    can_filter_t filter0 = {
        .bank_id      = 0,
        .filter_id    = TELEMETRY_CAN_ID,  /* 0x120 */
        .filter_mask  = CAN_STD_ID_MAX,    /* 0x7FF: Exact match required */
        .fifo_assign  = 0,
        .enabled      = true
    };
    can_driver_set_filter(&filter0);

    printf("[FILTER] Bank 0 Configured: ID=0x%03X, MASK=0x%03X -> Routed to FIFO 0\n",
           filter0.filter_id, filter0.filter_mask);
    print_separator();

    /* Simulated System State */
    uint32_t system_time_ms = 0;
    uint32_t last_rx_timestamp = 0;
    ev_telemetry_t tx_telemetry;
    ev_telemetry_t rx_telemetry;
    can_msg_t tx_frame;
    can_msg_t rx_frame;

    /* ---------------------------------------------------------------------- */
    /* Test Case 1: Normal Periodic Telemetry Transmission & Reception        */
    /* ---------------------------------------------------------------------- */
    printf("\n>>> TEST 1: Normal Periodic Telemetry Transmission (100ms Cycle)\n");
    tx_telemetry.pack_voltage_mv   = 52400; /* 52.4 V */
    tx_telemetry.motor_rpm         = 2450;  /* 2450 RPM */
    tx_telemetry.pack_temp_c       = 36;    /* 36 degC */
    tx_telemetry.vehicle_speed_kmh = 48;    /* 48 km/h */
    tx_telemetry.status_flags      = 0x01;  /* Inverter Ready */
    tx_telemetry.rolling_counter   = 1;

    system_time_ms += 100;
    telemetry_pack(&tx_telemetry, &tx_frame);
    print_can_frame("  [TX Node 1]", &tx_frame);

    can_driver_transmit(&tx_frame);

    /* Node 2 checks RX FIFO */
    if (can_driver_receive(&rx_frame) == CAN_OK) {
        print_can_frame("  [RX Node 2]", &rx_frame);
        telemetry_unpack(&rx_frame, &rx_telemetry);
        last_rx_timestamp = system_time_ms;

        printf("  [DECODED] Pack: %.1fV | Motor: %d RPM | Temp: %d C | Speed: %d km/h | Seq: %d\n",
               rx_telemetry.pack_voltage_mv / 1000.0f, rx_telemetry.motor_rpm,
               rx_telemetry.pack_temp_c, rx_telemetry.vehicle_speed_kmh,
               rx_telemetry.rolling_counter);
        printf("  [HEALTH] Status: NORMAL | Faults: 0x%02X (Clean)\n", rx_telemetry.active_faults);
    }

    /* ---------------------------------------------------------------------- */
    /* Test Case 2: Hardware Filter Rejection (Noise / Foreign Frame)          */
    /* ---------------------------------------------------------------------- */
    printf("\n>>> TEST 2: Hardware Acceptance Filter Validation (Foreign ID: 0x345)\n");
    can_msg_t foreign_frame = {
        .id          = 0x345, /* Unauthorized broadcast ID */
        .is_extended = false,
        .is_rtr      = false,
        .dlc         = 4,
        .data        = {0xDE, 0xAD, 0xBE, 0xEF}
    };
    print_can_frame("  [BUS INJECT]", &foreign_frame);
    can_driver_transmit(&foreign_frame);

    /* Verify whether the frame bypassed the hardware filter */
    if (can_driver_receive(&rx_frame) == CAN_OK) {
        printf("  [FAIL] Foreign frame 0x345 leaked into FIFO!\n");
    } else {
        printf("  [PASS] Silicon Filter Rejected ID 0x345: Zero CPU Interrupt Overhead!\n");
    }

    /* ---------------------------------------------------------------------- */
    /* Test Case 3: Safety Alarm Triggering (Cell Over-Temperature > 60 C)    */
    /* ---------------------------------------------------------------------- */
    printf("\n>>> TEST 3: Thermal Safety Threshold Triggering (Temp: 64 C > 60 C)\n");
    tx_telemetry.pack_temp_c     = 64;   /* Exceeds 60 C safety threshold */
    tx_telemetry.rolling_counter = 2;

    system_time_ms += 100;
    telemetry_pack(&tx_telemetry, &tx_frame);
    print_can_frame("  [TX Node 1]", &tx_frame);
    can_driver_transmit(&tx_frame);

    if (can_driver_receive(&rx_frame) == CAN_OK) {
        telemetry_unpack(&rx_frame, &rx_telemetry);
        last_rx_timestamp = system_time_ms;

        if (rx_telemetry.active_faults & FAULT_OVER_TEMP) {
            printf("  [CRITICAL ALARM] >>> BATTERY OVER-TEMP DETECTED (%d C)! <<<\n",
                   rx_telemetry.pack_temp_c);
            printf("  [ACTION] Activating Cooling Pump & Derating Motor Inverter!\n");
        }
    }

    /* ---------------------------------------------------------------------- */
    /* Test Case 4: Heartbeat Communication Loss Timeout (300ms Limit)        */
    /* ---------------------------------------------------------------------- */
    printf("\n>>> TEST 4: Communication Timeout Detection (Heartbeat Loss > 300ms)\n");
    /* Simulate 350ms passing with no CAN transmission (bus wire disconnected) */
    system_time_ms += 350;

    if (telemetry_is_timed_out(last_rx_timestamp, system_time_ms)) {
        printf("  [FAULT] No CAN heartbeat received for %u ms (Limit: %u ms)\n",
               system_time_ms - last_rx_timestamp, TELEMETRY_TIMEOUT_LIMIT_MS);
        printf("  [FAIL-SAFE] >>> CAN_COMMS_LOSS_DTC Raised - Transitioning to Safe State <<<\n");
    }

    /* ---------------------------------------------------------------------- */
    /* Summary Driver Statistics                                              */
    /* ---------------------------------------------------------------------- */
    print_separator();
    can_stats_t stats;
    can_driver_get_stats(&stats);
    printf("| STATS: TX Success: %d | RX Accepted: %d | Hardware Filter Dropped: %d    |\n",
           stats.tx_success_count, stats.rx_filtered_count, stats.rx_dropped_count);
    print_separator();
    printf("\n[SUCCESS] All CAN 2.0B Prototype Scenarios Executed Successfully.\n");

    return 0;
}
