# Capacitive Sensor Network Synchronization Project

This project implements a multi-node capacitive sensing network using **STM32L412KB** microcontrollers and **XBee** radios.  
The system acquires synchronized measurements from single-plate capacitive sensors based on the **drift-rejection differential frontend** described in the IEEE Sensors Journal paper *"Drift Rejection Differential Frontend for Single-Plate Capacitive Sensors"* (Subbicini, Lavagno, Lazarescu, 2022).  
The project was supervised by **Professor Mihai Teodor Lazarescu** (Politecnico di Torino).

The objective is to collect stable, real-time capacitance data from multiple sensor nodes using ID-based time-division synchronization, coordinated by a MATLAB server over serial communication.

---

## Project Structure

```
Capacitive_Sensor_Sync/
├── keil_firmware/          # STM32 firmware (Keil µVision 5 project)
│   ├── Core/
│   │   ├── Inc/            # Header files (main.h, HAL config, interrupts)
│   │   └── Src/            # Source files (main.c, HAL MSP, interrupts, system)
│   ├── Drivers/            # STM32 HAL & CMSIS libraries
│   ├── MDK-ARM/            # Keil project files & build output
│   └── chip_configuration.ioc  # STM32CubeMX pin/peripheral config
├── matlab_server/          # MATLAB script to control timing and receive sensor data
│   └── server_script.m
└── Useful_Material/        # Reference documents
    ├── fediff.pdf                                          # Sensor frontend circuit schematic
    └── Drift_Rejection_Differential_Frontend_for_...pdf    # IEEE research article
```

---

## Hardware

| Component | Details |
|---|---|
| **MCU** | STM32L412KBU6 (UFQFPN-32, Cortex-M4, STM32L4 family) |
| **Radio** | XBee module (UART-based, API mode) |
| **Sensor Frontend** | Drift-rejection differential circuit (see `fediff.pdf`) |
| **IDE** | Keil µVision 5 (MDK-ARM) |
| **Config Tool** | STM32CubeMX v6.16.1 |

---

## Peripherals Used

The following STM32 peripherals are configured and used in the firmware (`main.c`):

| Peripheral | Role | Configuration |
|---|---|---|
| **ADC1** | Reads the analog output of the capacitive sensor frontend | 12-bit resolution, single-ended on **Channel 5** (PA0), externally triggered by **TIM2 TRGO**, DMA-driven |
| **DMA1 Channel 1** | Transfers ADC samples into a memory buffer without CPU intervention | Peripheral-to-memory, half-word alignment, normal mode (single-frame capture) |
| **TIM1** | Generates the 1 kHz excitation square wave for the sensor frontend | Prescaler = 79, Period = 999 → 1 kHz PWM on **CH3N** (PB1); also acts as master trigger (TRGO = Update) to TIM2 |
| **TIM2** | Produces the 2 MHz ADC sampling clock, phase-locked to TIM1 | Prescaler = 0, Period = 39 → 2 MHz; slaved to TIM1 via combined reset + trigger (ITR0); TRGO triggers ADC1 |
| **USART1** | Serial link to the **XBee** radio module | 115200 baud, 8N1, TX/RX on PA9/PA10 |
| **USART2** | Debug / PC serial port (ST-Link VCP) | 115200 baud, 8N1, TX/RX on PA2/PA15 |
| **GPIO (PB7)** | XBee sleep/wake control pin | Push-pull output, used to toggle XBee ON/SLEEP |
| **SysTick** | HAL timebase for `HAL_Delay()` and timeout management | Default 1 ms tick |
| **NVIC** | Interrupt controller | DMA1_Ch1 (priority 0), USART1 & USART2 RX interrupts (priority 5) |

### Pin Mapping Summary

| Pin | Function |
|---|---|
| PA0 | ADC1_IN5 (sensor analog input) |
| PA2 | USART2_TX |
| PA9 | USART1_TX (XBee) |
| PA10 | USART1_RX (XBee) |
| PA13 | SWDIO (debug) |
| PA14 | SWCLK (debug) |
| PA15 | USART2_RX |
| PB1 | TIM1_CH3N (PWM excitation output) |
| PB7 | GPIO Output (XBee ON/SLEEP) |

---

## Firmware Logic Overview

1. **Initialization** — System clock configured via PLL (MSI 16 MHz → 80 MHz SYSCLK). All peripherals are initialized via HAL.
2. **Boot message** — A `"Boot"` string is sent on both USART1 (XBee) and USART2 (debug) to confirm startup.
3. **Idle / Wait for command** — The MCU enters a loop waiting for the `'R'` (or `'r'`) character from USART1 (XBee), which is sent by the MATLAB server as a "ready" command.
4. **Measurement cycle** — Upon receiving the command:
   - DMA, TIM1, and TIM2 are re-initialized and started.
   - ADC collects a full frame of 2000 samples (2 × NR) via DMA.
   - TIM1 generates the PWM excitation; TIM2 provides the 2 MHz ADC trigger clock.
   - When the DMA transfer completes (`HAL_ADC_ConvCpltCallback`), the `adcFrameReady` flag is set.
5. **Capacitance computation** — The `compute_cp_farad()` function implements the slope-modulation algorithm from the reference paper, computing capacitance (Cp) from four averaged ADC segments.
6. **Result transmission** — The computed Cp value (in picofarads) and the sensor ID are formatted as a string and transmitted via USART1 to the XBee network, with a configurable ID-based delay (`SENSOR_ID_DELAY_MS`) for time-division multiplexing.

---

## MATLAB Server

The `server_script.m` script acts as the network coordinator:

- Opens a serial connection to the XBee coordinator at **115200 baud** on a configurable COM port.
- Sends a 2-byte ready command (`0xA0 0x00`) each cycle to trigger all sensor nodes.
- Waits up to 250 ms to collect responses from up to 4 sensors.
- Decodes the 2-byte sensor frames (sensor ID in upper nibble, 12-bit ADC value in lower 12 bits).
- Logs and prints per-cycle data for all sensors.

---

## Usage

### Prerequisites

- **Keil µVision 5** (MDK-ARM) with the STM32L4 device pack installed
- **STM32CubeMX** (optional — only needed to modify peripheral configurations)
- **ST-Link V2** debugger/programmer (or compatible)
- **MATLAB** (R2019b or later recommended) with the Instrument Control Toolbox
- **XBee modules** (configured in API mode, matching PAN ID and baud rate)

### Building & Flashing the Firmware

1. Open the Keil project file:
   ```
   keil_firmware/MDK-ARM/mihai_code1.uvprojx
   ```
2. **Set the Sensor ID** — In `Core/Src/main.c`, change the `SENSOR_ID` macro (1–4) to assign a unique ID to each node:
   ```c
   #define SENSOR_ID 1          // Change per node (1, 2, 3, or 4)
   #define SENSOR_ID_DELAY_MS 1000  // Stagger if needed
   ```
3. Build the project: **Project → Build Target** (or press `F7`).
4. Connect the STM32 board via ST-Link and flash: **Flash → Download** (or press `F8`).
5. Repeat for each sensor node with a different `SENSOR_ID`.

### Running the MATLAB Server

1. Connect the XBee coordinator to your PC via USB.
2. Open `matlab_server/server_script.m` in MATLAB.
3. Update the COM port to match your system:
   ```matlab
   port = "COM3";   % Change to your XBee coordinator's COM port
   ```
4. Run the script. It will:
   - Send a ready command each cycle
   - Print received capacitance data from all sensor nodes
   - Store per-sensor measurement vectors (`responses1` – `responses4`)

### Serial Debug (Optional)

You can monitor the debug USART2 output using any serial terminal (e.g., PuTTY, Tera Term) at **115200 baud, 8N1** to see boot messages and sensor readings.

---

## Key Configuration Constants

| Constant | Value | Description |
|---|---|---|
| `SENSOR_ID` | 1–4 | Unique node identifier for TDMA scheduling |
| `SENSOR_ID_DELAY_MS` | 1000 | Delay before transmitting (ms), used for time-slot offset |
| `NR` | 1000 | Samples per half-period (total frame = 2 × NR) |
| `NS` | 256 | Averaging window length (ADC codes) |
| `NG` | 64 | Guard samples at ramp start (transient rejection) |
| `I_FRONTEND` | 40 nA | Frontend excitation current |
| `TM_SEC` | 1 ms | TIM1 modulation period |

---

## Highlights

- Real-time synchronized data collection from up to 4 capacitive sensors
- Embedded C firmware using the STM32 HAL library
- XBee API-mode wireless communication for multi-node networking
- MATLAB-based serial server for coordinating sensor transmissions and logging data
- Hardware frontend capable of **rejecting drift noise** and enabling long-range (~2 m) capacitive sensing
- Slope-modulation capacitance extraction algorithm implemented on-chip

---

## References

- A. Subbicini, L. Lavagno, M. T. Lazarescu, *"Drift Rejection Differential Frontend for Single-Plate Capacitive Sensors,"* IEEE Sensors Journal, 2022.
- See `Useful_Material/` for the full paper and frontend schematic.
