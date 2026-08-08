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
│   │   ├── Inc/            # main.h, stm32l4xx_hal_conf.h, stm32l4xx_it.h
│   │   └── Src/            # main.c, stm32l4xx_hal_msp.c, stm32l4xx_it.c, system_stm32l4xx.c
│   ├── Drivers/            # STM32L4 HAL + CMSIS (trimmed — see "Repository Hygiene")
│   ├── MDK-ARM/            # Keil project files, startup_stm32l412xx.s, build output
│   └── chip_configuration.ioc  # STM32CubeMX pin/peripheral config
├── matlab_server/          # MATLAB coordinator script
│   └── server.m
└── Useful_Material/        # Reference documents
    ├── fediff.pdf                                          # Sensor frontend circuit schematic
    └── Drift_Rejection_Differential_Frontend_for_...pdf    # IEEE research article
```

---

## Hardware

| Component | Details |
|---|---|
| **MCU** | STM32L412KBU6 (UFQFPN-32, Cortex-M4, STM32L4 family) |
| **Radio** | XBee module (UART-based) |
| **Sensor Frontend** | Drift-rejection differential circuit (see `fediff.pdf`) |
| **IDE** | Keil µVision 5 (MDK-ARM) |
| **Config Tool** | STM32CubeMX v6.16.1 |

Clock tree: MSI (range 8) → PLL (M=1, N=10, R=/2) → **80 MHz SYSCLK**, AHB/APB1/APB2 dividers = 1, Flash latency 4, voltage scale 1.

---

## Peripherals Used

| Peripheral | Role | Configuration |
|---|---|---|
| **ADC1** | Reads the analog output of the capacitive sensor frontend | 12-bit, single-ended, **Channel 5** (PA0), sampling time 2.5 cycles, externally triggered by **TIM2 TRGO** (rising edge), DMA requests enabled, overrun = data preserved. Self-calibrated **once at boot** via `HAL_ADCEx_Calibration_Start()` |
| **DMA1 Channel 1** | Moves ADC samples into `adcBuf[2000]` without CPU intervention | Request 0, peripheral→memory, half-word alignment, memory increment, **normal mode** (one-shot frame of `FRAME = 2 × NR = 2000` samples) |
| **TIM1** | Generates the 1 kHz excitation square wave for the frontend | Prescaler 79, Period 999 → 1 kHz; PWM1 on **CH3N** (PB1) at 50 % duty (Pulse = 500). Master, TRGO = Update → drives TIM2 |
| **TIM2** | Produces the 2 MHz ADC sampling clock, phase-locked to TIM1 | Prescaler 0, Period 39 → 2 MHz; slave in **combined reset+trigger** mode on **ITR0** (TIM1); TRGO = Update → triggers ADC1 |
| **USART1** | Serial link to the **XBee** radio (binary protocol) | 115200 baud, 8N1, TX/RX on PA9/PA10, polling mode (`HAL_UART_Receive` / `HAL_UART_Transmit`) |
| **USART2** | Optional debug port (ST-Link VCP) — compiled in only when `DEBUG_UART_ENABLED == 1` | 115200 baud, 8N1, TX on PA2, RX on PA15 |
| **GPIO PB7** | XBee `SLEEP_RQ` control | Push-pull output, driven **low = awake**; latch is cleared before the pin is switched to output so no high pulse appears at startup |
| **SysTick** | HAL timebase for `HAL_Delay()` and the acquisition timeout | 1 ms tick |
| **NVIC** | Interrupt controller | **Only `DMA1_Channel1_IRQn` is enabled** (priority 0, 0). The USART IRQ handlers exist in `stm32l4xx_it.c` but their NVIC lines are not enabled — UART is polled |

### Pin Mapping

| Pin | Function |
|---|---|
| PA0 | ADC1_IN5 (sensor analog input) |
| PA2 | USART2_TX (debug, AF7) |
| PA9 | USART1_TX (XBee, AF7) |
| PA10 | USART1_RX (XBee, AF7) |
| PA13 | SWDIO (debug) |
| PA14 | SWCLK (debug) |
| PA15 | USART2_RX (debug, AF3) |
| PB1 | TIM1_CH3N (PWM excitation output, AF1) |
| PB7 | GPIO output (XBee SLEEP_RQ) |

---

## Communication Protocol

### Request (MATLAB → nodes)

The MATLAB server writes a single byte over serial; the XBee coordinator broadcasts it to every node.
`server.m` sends lowercase **`'r'`**; the firmware accepts **either `'r'` or `'R'`**.

### Response (node → MATLAB)

Each node replies with a **2-byte big-endian frame**:

```
Byte 0 (MSB):  [ ID1 | ID0 | P13 | P12 | P11 | P10 | P9 | P8 ]
Byte 1 (LSB):  [ P7  | P6  | P5  | P4  | P3  | P2  | P1 | P0 ]
```

| Field | Bits | Description |
|---|---|---|
| **Sensor ID** | `[15:14]` | Node identifier, 0–3 |
| **Payload** | `[13:0]` | Capacitance in units of **0.1 pF** |

Payload encoding:

| Payload | Meaning |
|---|---|
| `0x0000` – `0x3FFE` | Valid measurement, 0 … 1638.2 pF (`Cp_pF = payload / 10`) |
| `0x3FFF` | **Error sentinel** — acquisition or computation failed |

Valid results are deliberately clamped at `0x3FFE` so `0x3FFF` stays reserved. A node **always** answers, even on failure, so the server can distinguish "node reported an error" from "node never replied".

**Encoding (firmware, `pack_measurement()`):**

```c
float scaled = cp_farad * 1.0e12f * CAP_SCALE_PER_PF;   /* farads → 0.1 pF units */
payload = (uint16_t)(scaled + 0.5f);                    /* round, then clamp to 0x3FFE */
return (uint16_t)(((SENSOR_ID & 0x0003U) << 14) | (payload & 0x3FFFU));
```

**Decoding (MATLAB, `server.m`):**

```matlab
packet        = bitor(bitshift(uint16(bytes(1)), 8), uint16(bytes(2)));
transmitterID = double(bitand(bitshift(packet, -14), uint16(3)));
sensorValue   = double(bitand(packet, uint16(hex2dec("3FFF"))));
```

### Collision avoidance

The broadcast reaches all nodes at nearly the same instant, so each node waits
`SENSOR_ID × RESPONSE_SLOT_MS` (10 ms per slot) **after** finishing its measurement and **before**
transmitting. With four nodes the responses land at roughly 0, 10, 20 and 30 ms.

---

## Firmware Logic Overview

1. **Boot** — `HAL_Init()`, `SystemClock_Config()` (80 MHz), then every `MX_*_Init()` runs **once**. Peripherals are started and stopped per measurement, but never re-initialized. `MX_USART2_UART_Init()` is compiled in only when `DEBUG_UART_ENABLED == 1`.
2. **XBee wake** — PB7 is driven low so the radio stays awake if pin-sleep is ever enabled.
3. **ADC calibration** — `HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)` runs once while the ADC is idle; failure calls `Error_Handler()`.
4. **Idle** — the node blocks in `HAL_UART_Receive(&huart1, …, HAL_MAX_DELAY)` waiting for one request byte. On a UART error it calls `HAL_UART_AbortReceive()` and re-arms the receiver instead of hanging.
5. **Acquisition** (`acquire_adc_frame()`) — on `'r'`/`'R'`:
   - TIM1 and TIM2 counters are zeroed and their update flags cleared, so every frame starts from the same timer phase.
   - Start order is **ADC+DMA → TIM2 base → TIM1 CH3N**. `HAL_TIMEx_PWMN_Start()` both enables the complementary output and starts TIM1, so no separate `HAL_TIM_Base_Start(&htim1)` is needed.
   - The loop waits for `adcFrameReady`, set by `HAL_ADC_ConvCpltCallback()`, with an **`ADC_FRAME_TIMEOUT_MS` = 20 ms** guard.
   - `stop_acquisition()` always runs (success, failure or timeout) and stops PWM, TIM2 and the DMA so the next one-shot transfer starts from a clean HAL state.
6. **Capacitance computation** (`compute_cp_farad()`) — slope-modulation extraction over four averaged windows of the 2000-sample frame:

   | Window | Start index | Length |
   |---|---|---|
   | `a1` | `NG` | `NS` |
   | `a2` | `NR − NG − NS` | `NS` |
   | `a3` | `NR + NG` | `NS` |
   | `a4` | `2·NR − NG − NS` | `NS` |

   Codes are converted to volts with `VREF / ADC_FS`, then

   ```
   falling  = a2 − a1
   rising   = a4 − a3
   denom    = rising − falling
   Cp       = (I_FRONTEND · TM_SEC / denom) · (NR − 2·NG − NS) / NR
   ```

   If `|denom| < 1 µV` the function returns `-1.0f` (invalid). The sign of `Cp` depends on the physical CH3N polarity, so the absolute value is taken.
7. **Response** — the node delays for its slot, packs ID + payload, and transmits 2 bytes on USART1. In debug builds `debug_report()` also prints `Cp=… pF, sensor ID=…` (or `Cp=ERROR …` with the HAL status) on USART2.

### Compile-time safety checks

```c
#if SENSOR_ID > 3U
#error "SENSOR_ID must fit in two bits (0..3)."
#endif

#if ((2U * NG + NS) >= NR)
#error "NR, NS and NG do not define valid slope-measurement segments."
#endif
```

---

## MATLAB Server (`matlab_server/server.m`)

Round-based coordinator. For each of `numberOfSamples` rounds it:

1. Flushes the input buffer to discard late bytes from the previous round.
2. Writes the request byte `'r'`.
3. Reads 2-byte packets until **all four IDs have answered** or `roundTimeout` expires.
4. Decodes ID and 14-bit payload, records the **arrival time in ms** relative to the request.
5. Ignores duplicate responses from an ID already seen in that round (counted in `duplicatePacketCount`).
6. Leaves missing transmitters as `NaN` and logs which IDs timed out.
7. Pauses `interRoundDelay` before the next round.

### Configuration

| Variable | Default | Description |
|---|---|---|
| `portName` | `"COM8"` | Serial port of the XBee coordinator |
| `baudRate` | `115200` | Must match the firmware and the XBee |
| `numberOfSamples` | `100` | Number of request rounds |
| `roundTimeout` | `0.20` s | Maximum wait for all four responses in a round |
| `interRoundDelay` | `0.010` s | Idle time between rounds |

### Outputs

Everything is saved to **`four_transmitter_results.mat`**:

| Variable | Shape | Contents |
|---|---|---|
| `transmitter0Values` … `transmitter3Values` | `numberOfSamples × 1` | Raw 14-bit payload per round, `NaN` if missing |
| `transmitter0Received` … `transmitter3Received` | `numberOfSamples × 1` logical | Reception mask (`~isnan`) |
| `transmitter0ArrivalMs` … `transmitter3ArrivalMs` | `numberOfSamples × 1` | Arrival time in ms since the request |
| `sensorValues` | `numberOfSamples × 4` | Columns = IDs 0, 1, 2, 3 |
| `arrivalTimeMs` | `numberOfSamples × 4` | Same layout, arrival times |
| `incompleteRoundCount`, `duplicatePacketCount`, `unexpectedPacketCount` | scalar | Diagnostic counters |
| `baudRate`, `roundTimeout` | scalar | Run configuration |

> **Note on units:** `sensorValues` holds the raw payload in 0.1 pF steps. Convert with `sensorValues / 10` to get pF.
> The script does **not** currently special-case the `0x3FFF` (16383) error sentinel — treat that value as an error rather than a 1638.3 pF reading, e.g. `sensorValues(sensorValues == 16383) = NaN;`

The `valueText()` local helper formats `NaN` as `"NaN"` in the per-round console output.

---

## Usage

### Prerequisites

- **Keil µVision 5** (MDK-ARM) with the STM32L4 device pack installed
- **STM32CubeMX** (optional — only to modify peripheral configuration via `chip_configuration.ioc`)
- **ST-Link V2** debugger/programmer (or compatible)
- **MATLAB** R2019b or later (uses the `serialport` interface)
- **XBee modules** configured with matching PAN ID, channel and 115200 baud

### Building & flashing

1. Open `keil_firmware/MDK-ARM/mihai_code1.uvprojx`.
2. **Set the sensor ID** — in `Core/Src/main.c`, give each node a unique value:
   ```c
   #define SENSOR_ID 1U         /* 0, 1, 2 or 3 — one per node */
   ```
3. **Debug output** (optional) — leave at `0` for deployment:
   ```c
   #define DEBUG_UART_ENABLED 0U   /* 1 = USART2 ASCII debug, 0 = XBee only */
   ```
4. Build with **Project → Build Target** (`F7`).
5. Flash via ST-Link with **Flash → Download** (`F8`).
6. Repeat for each node, changing only `SENSOR_ID`.

### Running the server

1. Connect the XBee coordinator to the PC.
2. Open `matlab_server/server.m` and set `portName` to your COM port (the script prints `serialportlist("available")` at startup to help).
3. Run it. Per-round console output looks like:
   ```
   ----- Round 7 -----
   ID=0, bytes=00 7B, packet=0x007B, value=  123, arrival=   2.14 ms
   ID=1, bytes=40 82, packet=0x4082, value=  130, arrival=  12.07 ms
   Round 7 timed out. Missing IDs: [2 3]
   Stored: ID0=123, ID1=130, ID2=NaN, ID3=NaN
   ```
4. A summary of rounds, incomplete rounds, duplicates and per-ID reception counts is printed at the end, then results are written to `four_transmitter_results.mat`.

### Serial debug (optional)

With `DEBUG_UART_ENABLED == 1`, attach a terminal (PuTTY, Tera Term) to the ST-Link VCP at **115200 8N1** to see the boot banner and human-readable `Cp` readings.

---

## Key Configuration Constants (`main.c`)

| Constant | Value | Description |
|---|---|---|
| `SENSOR_ID` | `1U` | Unique node identifier, 0–3, packed into the two MSBs |
| `RESPONSE_SLOT_MS` | `10U` | Per-ID transmit delay; node waits `SENSOR_ID × 10 ms` |
| `DEBUG_UART_ENABLED` | `0U` | Compiles in USART2 and ASCII debug output |
| `NR` | `1000U` | Samples per half-period |
| `FRAME` | `2 × NR = 2000` | Total samples per acquisition |
| `NS` | `256U` | Averaging window length |
| `NG` | `64U` | Guard samples rejected at each ramp edge |
| `ADC_FS` | `4095.0f` | 12-bit full scale |
| `VREF` | `3.300f` V | ADC reference |
| `I_FRONTEND` | `40e-9f` A | Frontend excitation current |
| `TM_SEC` | `1.0e-3f` s | TIM1 modulation period (1 kHz) |
| `ADC_FRAME_TIMEOUT_MS` | `20U` | Guard against a DMA frame that never completes |
| `CAP_SCALE_PER_PF` | `10.0f` | Payload resolution: 10 counts per pF (0.1 pF/LSB) |
| `CAP_PAYLOAD_MAX` | `0x3FFE` | Largest valid payload (1638.2 pF) |
| `CAP_PAYLOAD_ERROR` | `0x3FFF` | Reserved error sentinel |
| `XBEE_SLEEP_GPIO_PORT` / `XBEE_SLEEP_PIN` | `GPIOB` / `GPIO_PIN_7` | XBee `SLEEP_RQ` line |

---

## Repository Hygiene

The vendor `Drivers/` tree shipped by STM32CubeMX is ~97 MB, almost all of it unused. Only the
files this project actually compiles or includes are tracked; the rest are listed in `.gitignore`.

**Kept:**

- `Drivers/STM32L4xx_HAL_Driver/Inc/` (and `Inc/Legacy/`) — full header set, since HAL headers cross-include each other
- `Drivers/STM32L4xx_HAL_Driver/Src/` — only the 21 `.c` files in the Keil build (hal, adc, adc_ex, rcc, rcc_ex, flash, flash_ex, flash_ramfunc, gpio, i2c, i2c_ex, dma, dma_ex, pwr, pwr_ex, cortex, exti, tim, tim_ex, uart, uart_ex)
- `Drivers/CMSIS/Include/` — Cortex core headers (on the compiler include path)
- `Drivers/CMSIS/Device/ST/STM32L4xx/Include/` — only `stm32l412xx.h`, `stm32l4xx.h`, `system_stm32l4xx.h`

**Ignored:** `CMSIS/DSP`, `CMSIS/NN`, `CMSIS/RTOS`, `CMSIS/RTOS2`, `CMSIS/Core`, `CMSIS/docs`,
`CMSIS/Device/.../Source/Templates` (the project uses its own `MDK-ARM/startup_stm32l412xx.s`),
the other 24 STM32L4 device headers, and the 84 unused HAL/LL source files —
1100 files in all, leaving 168 tracked under `Drivers/`.

If you enable another peripheral in CubeMX, add its `.c` back with a negation line in `.gitignore`
and add it to the Keil project's file list.

---

## Highlights

- Event-driven nodes: idle in a blocking UART read, measure only on request
- One-shot DMA acquisition with deterministic timer phase and a hardware-independent timeout
- ID-based time-division slots (10 ms apart) to avoid radio collisions on a broadcast
- Compact 2-byte binary protocol with a reserved error code, so a failed node is still distinguishable from a silent one
- Peripherals initialized once and merely started/stopped per measurement
- Slope-modulation capacitance extraction running on-chip
- MATLAB coordinator with duplicate detection, arrival-time logging and per-round diagnostics
- Hardware frontend capable of **rejecting drift noise** and enabling long-range (~2 m) capacitive sensing

---

## References

- A. Subbicini, L. Lavagno, M. T. Lazarescu, *"Drift Rejection Differential Frontend for Single-Plate Capacitive Sensors,"* IEEE Sensors Journal, 2022.
- See `Useful_Material/` for the full paper and frontend schematic.
