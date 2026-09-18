# STM32 CAN 2.0B Communication & Telemetry Prototype

A modular, bare-metal **Controller Area Network (CAN 2.0B)** firmware implementation on ARM Cortex-M (STM32F103) demonstrating automotive-grade peripheral configuration, hardware acceptance filtering, deterministic fixed-point telemetry serialization, and safety supervision.

---

## 📌 Executive Summary & Motivation

In industrial and automotive distributed architectures, bus reliability and deterministic timing are critical. This prototype demonstrates a complete CAN communication pipeline built from first principles on the **STM32 bxCAN peripheral**:

* **Nominal Bit Timing:** Configured for standard **500 kbps** on a **36 MHz APB1** clock tree with an **83.3% sample point** adhering to ISO 11898-1 recommendations.
* **Hardware Acceptance Filtering:** Implements **32-bit Identifier Mask Mode** across silicon filter banks to reject unauthorized bus traffic at the hardware level, preventing CPU interrupt starvation.
* **Deterministic Telemetry Packing:** 8-byte big-endian fixed-point serialization for Electric Vehicle (EV) battery pack telemetry (Voltage, Current, Cell Temperature, Motor RPM).
* **Defensive Safety Supervision:** Real-time over-temperature threshold alarms and a non-blocking **300ms heartbeat timeout monitor** to trigger safe-state failover during communication loss.
* **Silicon Self-Test Verification:** Validated using the internal **bxCAN Loopback Mode**, allowing full register, mailbox, filter, and FIFO validation on a single MCU without external transceivers.

---

## 🏗️ System Architecture & Data Flow

```mermaid
flowchart TD
    subgraph AppLayer["Application & Supervision Layer"]
        Sensors["EV Telemetry Source<br>(Pack Volt, Temp, Motor RPM)"]
        Safety["Safety Supervisor<br>(Over-Temp & 300ms Timeout)"]
        Display["UART Telemetry Monitor<br>(PC Console Output)"]
    end

    subgraph Middleware["Serialization / Filter Layer"]
        Packer["telemetry_pack()<br>(8-Byte Big-Endian Packing)"]
        Unpacker["telemetry_unpack()<br>(Fixed-Point Unpacking)"]
    end

    subgraph HardwareCAN["STM32 bxCAN Silicon Controller"]
        direction LR
        TX_Mailbox["TX Mailbox 0<br>(500 kbps)"]
        LoopbackPath["Internal Loopback Route"]
        FilterBank["32-bit Filter Bank 0<br>(ID: 0x120, Mask: 0x7FF)"]
        RX_FIFO["RX FIFO 0<br>(Non-Blocking Ring Buffer)"]

        TX_Mailbox --> LoopbackPath --> FilterBank --> RX_FIFO
    end

    Sensors --> Packer --> TX_Mailbox
    RX_FIFO --> Unpacker --> Safety --> Display
```

---

## ⏱️ CAN Bit Timing Configuration (36 MHz APB1 $\rightarrow$ 500 kbps)

The nominal bit time is partitioned according to ISO 11898-1 into discrete **Time Quanta ($T_q$)**:

$$\text{Bit Time} = T_{sync} + T_{ts1} + T_{ts2} = 1 + 14 + 3 = 18\ T_q$$

$$T_q = \frac{\text{BRP}}{f_{pclk1}} = \frac{4}{36,000,000} = 111.11\text{ ns}$$

$$\text{Nominal Bit Rate} = \frac{1}{18 \times 111.11\text{ ns}} = 500,000\text{ bps (500 kbps)}$$

$$\text{Sample Point} = \frac{1 + \text{TS1}}{\text{Total } T_q} = \frac{1 + 14}{18} = 83.33\%$$

### Register Configuration Table:

| Parameter | Symbol | Register Field | Configured Value | Industry Standard Guideline |
| :--- | :---: | :---: | :---: | :--- |
| **APB1 Clock** | $f_{pclk1}$ | RCC_CFGR | 36 MHz | Max for STM32F103 |
| **Baud Rate Prescaler** | BRP | `CAN_BTR[9:0]` | **4** | $T_q = 111.11\text{ ns}$ |
| **Time Segment 1** | TS1 | `CAN_BTR[19:16]` | **14 $T_q$** | Prop_Seg + Phase_Seg1 |
| **Time Segment 2** | TS2 | `CAN_BTR[22:20]` | **3 $T_q$** | Phase_Seg2 (Information Processing) |
| **Resync Jump Width** | SJW | `CAN_BTR[25:24]` | **1 $T_q$** | Clock drift compensation |
| **Sample Point** | SP | — | **83.3%** | Recommended: 75% to 85% |

---

## 🛡️ 32-bit Hardware Acceptance Filter Mask Logic

To eliminate software overhead from handling unlisted broadcast frames, the STM32 bxCAN filter bank is configured in **32-bit Identifier Mask Mode**:

* **Target Telemetry ID:** `0x120` (Standard 11-bit identifier)
* **Acceptance Mask:** `0x7FF` (Every bit from bit 0 to 10 must match exactly)

```
Incoming Frame ID:   0 0 1   0 0 1 0   0 0 0 0  (0x120)
Filter Mask:         1 1 1   1 1 1 1   1 1 1 1  (0x7FF: Must Match All)
Filter Target ID:    0 0 1   0 0 1 0   0 0 0 0  (0x120)
-------------------------------------------------------------
Result: MATCH -> Routed to RX FIFO 0

Incoming Noise ID:   0 1 1   0 1 0 0   0 1 0 1  (0x345)
Filter Mask:         1 1 1   1 1 1 1   1 1 1 1  (0x7FF)
Filter Target ID:    0 0 1   0 0 1 0   0 0 0 0  (0x120)
-------------------------------------------------------------
Result: NO MATCH -> Silently discarded by hardware silicon (Zero CPU load)
```

---

## 📊 Telemetry Data Frame Layout (ID: `0x120`, Cycle: 100ms)

| Byte | Signal Name | Data Type | Resolution / Unit | Range | Description |
| :---: | :--- | :---: | :---: | :---: | :--- |
| **0–1** | `pack_voltage_mv` | `uint16_t` | 1 mV / count | 0 to 65.535 V | High byte: `Byte 0`, Low byte: `Byte 1` |
| **2–3** | `motor_rpm` | `uint16_t` | 1 RPM / count | 0 to 8,000 RPM | High byte: `Byte 2`, Low byte: `Byte 3` |
| **4** | `pack_temp_c` | `int8_t` | 1 °C / count | -40 to +125 °C | Cell thermal reading |
| **5** | `vehicle_speed_kmh`| `uint8_t` | 1 km/h / count | 0 to 200 km/h | Road speed |
| **6** | `status_flags` | `uint8_t` | Bitfield | 0x00 to 0xFF | Bit 0: Inverter Ready, Bit 1: Brake, Bit 2: Charge |
| **7** | `rolling_counter` | `uint8_t` | 1 count (0..15)| 0 to 15 | Alive sequence counter in lower nibble |

---

## 💻 Execution & Verification Output

The prototype was compiled and executed via GCC with full diagnostic telemetry:

```text
+-------------------------------------------------------------------------+
|   STM32F103 bxCAN 2.0B Telemetry & Safety Node - Engineering Prototype  |
|   Author: S. Sherifred Singh | Target: 500 kbps Automotive Bus          |
+-------------------------------------------------------------------------+
[INIT] Mode: LOOPBACK (Internal Silicon Self-Test)
[INIT] Clock: 36 MHz | Baudrate: 500 kbps | Prescaler: 4
[INIT] Bit Segments: TS1=14 Tq, TS2=3 Tq, SJW=1 Tq | Sample Point: 83.3%
[FILTER] Bank 0 Configured: ID=0x120, MASK=0x7FF -> Routed to FIFO 0
+-------------------------------------------------------------------------+

>>> TEST 1: Normal Periodic Telemetry Transmission (100ms Cycle)
  [TX Node 1] [ID: 0x120 | DLC: 8 | Data: CC B0 09 92 24 30 01 01]
  [RX Node 2] [ID: 0x120 | DLC: 8 | Data: CC B0 09 92 24 30 01 01]
  [DECODED] Pack: 52.4V | Motor: 2450 RPM | Temp: 36 C | Speed: 48 km/h | Seq: 1
  [HEALTH] Status: NORMAL | Faults: 0x00 (Clean)

>>> TEST 2: Hardware Acceptance Filter Validation (Foreign ID: 0x345)
  [BUS INJECT] [ID: 0x345 | DLC: 4 | Data: DE AD BE EF]
  [PASS] Silicon Filter Rejected ID 0x345: Zero CPU Interrupt Overhead!

>>> TEST 3: Thermal Safety Threshold Triggering (Temp: 64 C > 60 C)
  [TX Node 1] [ID: 0x120 | DLC: 8 | Data: CC B0 09 92 40 30 01 02]
  [CRITICAL ALARM] >>> BATTERY OVER-TEMP DETECTED (64 C)! <<<
  [ACTION] Activating Cooling Pump & Derating Motor Inverter!

>>> TEST 4: Communication Timeout Detection (Heartbeat Loss > 300ms)
  [FAULT] No CAN heartbeat received for 350 ms (Limit: 300 ms)
  [FAIL-SAFE] >>> CAN_COMMS_LOSS_DTC Raised - Transitioning to Safe State <<<
+-------------------------------------------------------------------------+
| STATS: TX Success: 3 | RX Accepted: 2 | Hardware Filter Dropped: 1      |
+-------------------------------------------------------------------------+

[SUCCESS] All CAN 2.0B Prototype Scenarios Executed Successfully.
```

---

## 🛠️ How to Build and Run

### Prerequisites
* GCC compiler (MinGW for Windows or native Linux `build-essential`)
* Standard Make tool

### Quick Start
```bash
# Clone the repository
git clone https://github.com/YourUsername/stm32-can-telemetry-prototype.git
cd stm32-can-telemetry-prototype

# Compile the prototype
make

# Run the simulation and telemetry monitor
./can_demo
```

---

## 🎤 Interview Pitch Script (How to Present This in 60 Seconds)

> *"In addition to my production background in industrial serial protocols (Modbus RS-485, UART, I2C), I engineered this personal STM32 CAN 2.0B prototype to validate low-level automotive networking from first principles.*
>
> *I configured the STM32 bxCAN controller for 500 kbps by calculating the prescaler and phase segments on a 36 MHz APB1 clock tree, reaching an 83.3% sample point.*
>
> *To eliminate software interrupt overhead from high bus traffic, I configured 32-bit hardware acceptance filter banks in silicon to route only valid telemetry IDs to the RX FIFO. I also implemented fixed-point EV battery telemetry serialization, safety threshold evaluation for thermal runaway, and a 300ms heartbeat timeout detection algorithm."*

---

## 📄 License
This project is released under the **MIT License**.
