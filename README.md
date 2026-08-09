# Capacitive Sensor Sync

A 4-node wireless sensor network that measures capacitance with drift noise cancellation. when a person moves near a plate its capacitance changes very slightly, and the sensors track capacitance value and send it via XBee radio modules to the server side, in order to use the data for presence detection.

Each node is an **STM32L412KB** microcontroller with a **Digi XBee** radio. A **MATLAB** script on a PC broadcast a request message to all four nodes for a reading at the same moment and collects the answers. The method is from the paper : "Drift Rejection Differential Frontend for Single Plate Capacitive Sensors," *IEEE Sensors Journal* by G. Subbicini, L. Lavagno and M. T. Lazarescu, and this project was supervised by Prof. Mihai Teodor Lazarescu at Politecnico di Torino.

---

## How it fits together

![System architecture](docs/figures/system_architecture.svg)

The PC connects to a USB module **Waspmote Gateway**, which carries the coordinator XBee radio and presents it to MATLAB as an ordinary COM port. The radio then broadcasts the ready message to the four sensor nodes over the air.

## How one measurement works

1. MATLAB broadcasts a single letter, `r` 
2. Each node wakes up, measures and gets a capacitance value. This takes about a millisecond with this configuration of the ADC.
3. Each node waits its turn, which depends on their ID, then sends a two-byte answer.
4. MATLAB stores the data by the 2 msb bits indicating the node ID.

![Response time slots](docs/figures/response_slots.svg)

## How a node measures capacitance

The node drives its plate with a 1 kHz square wave and watches the plate voltage ramp up and down in response. How *steeply* the voltage ramps tells you the capacitance - a bigger capacitance charges more slowly.

The catch is drift. Slow leakage currents in the circuit bend both ramps in the same direction, and that drift can easily be larger than the tiny signal of our measurement. The idea from the paper is to compare the two ramps against each other rather than measure either one on its own. Drift affects both equally and cancels out.

In practice the node captures 2000 voltage samples per cycle, averages four fixed slices of them, and solves for capacitance. And the final number is sent via Xbee.

![ADC frame timing](docs/figures/adc_frame_timing.svg)

The sampling has to line up with the square wave the same way every single time, otherwise those four fixed slices would drift across the waveform and the cancelling method would not work. Two hardware timers handle this: one generates the square wave, the other paces the sampling and is reset by the first at every cycle. The [reference document](docs/configuration_reference.pdf) explains this in detail.

## What comes back

Each answer is two bytes. The top two bits indicate the node ID; the remaining fourteen bits carry the capacitance in steps of 0.1 pF.

---

## What's in the repository

| Folder | Contents |
|---|---|
| `keil_firmware/` | The STM32 firmware, as a Keil µVision 5 project |
| `matlab_server/` | `server.m`, the script that runs on the PC |
| `docs/` | The configuration reference document and its figures |
| `Useful_Material/` | The source paper and the sensor frontend schematic |


## Hardware

| | |
|---|---|
| Microcontroller | STM32L412KBU6 (Arm Cortex-M4, 80 MHz) |
| Node radio | Digi XBee 2.4 GHz, 802.15.4 firmware, 115200 baud over UART |
| PC interface | Waspmote Gateway USB module, carrying the coordinator radio |
| Sensor frontend | Drift-rejection differential circuit — schematic in `Useful_Material/fediff.pdf` |
| Tools | Keil µVision 5, STM32CubeMX, Digi XCTU, MATLAB |

---

## Getting it running

### Flash the nodes

1. Open `keil_firmware/MDK-ARM/mihai_code1.uvprojx` in Keil.
2. In `Core/Src/main.c`, give this node a unique number:

   ```c
   #define SENSOR_ID 1U      // 0, 1, 2 or 3 — different for every node
   ```
3. Build and flash over ST-Link.
4. Repeat for each node, changing only `SENSOR_ID`.

For readable debug output on terminal, set `DEBUG_UART_ENABLED` to `1` and open a serial terminal on the ST-Link COM port at 115200 baud.

### Set up the radios

Every radio is configured in Digi XCTU before it goes into a board — same PAN ID, same channel, 802.15.4 firmware, transparent mode, 115200 baud. The Waspmote gateway doubles as the programming jig: seat each module in it and write the settings through XCTU. The exact values, including pins connections on the PCB, are in [the reference document](docs/configuration_reference.pdf).

### Run the server

1. Plug the Waspmote gateway, with the coordinator radio seated in it, into the PC.
2. Open `matlab_server/server.m` and set `portName` to your COM port. The script lists the available ports when it starts.
3. the output looks like this :

   ```
   ----- Round 7 -----
   ID=0, bytes=00 7B, packet=0x007B, value=  123, arrival=   2.14 ms
   ID=1, bytes=40 82, packet=0x4082, value=  130, arrival=  12.07 ms
   Round 7 timed out. Missing IDs: [2 3]
   Stored: ID0=123, ID1=130, ID2=NaN, ID3=NaN
   ```

4. When it finishes, everything is saved to `four_transmitter_results.mat`.


## Reference

G. Subbicini, L. Lavagno and M. T. Lazarescu, "Drift Rejection Differential Frontend for Single Plate Capacitive Sensors," *IEEE Sensors Journal*, vol. 22, no. 16, pp. 16141–16149, 15 Aug. 2022. [doi:10.1109/JSEN.2022.3189031](https://doi.org/10.1109/JSEN.2022.3189031)

The full paper and the frontend schematic are in `Useful_Material/`.
