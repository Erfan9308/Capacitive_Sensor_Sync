# Capacitive Sensor Network Synchronization Project

This project implements a multi-node capacitive sensing network using STM32L4 microcontrollers and XBee radios.  
The system acquires synchronized measurements from single-plate capacitive sensors based on the **drift-rejection differential frontend** described in the IEEE Sensors Journal paper *“Drift Rejection Differential Frontend for Single-Plate Capacitive Sensors”* (Subbicini, Lavagno, Lazarescu, 2022).  
The project was supervised by **Professor Mihai Teodor Lazarescu** (Politecnico di Torino).

The objective is to collect stable, real-time capacitance data from multiple sensor nodes using ID-based time-division synchronization, coordinated by a MATLAB server over serial communication.

## Structure

- `keil_firmware/`: STM32 firmware (Keil uVision5 project)
- `matlab_server/`: MATLAB script to control timing and receive sensor data
- `Useful_Material/` —  
  Contains the sensor frontend circuit schematic (`fediff.pdf`) and the related IEEE research article  
  *“Drift Rejection Differential Frontend for Single-Plate Capacitive Sensors”*,  
  describing the slope-modulation measurement technique used in this project.

## Highlights

- Real-time synchronized data collection from 4 capacitive sensors
- Embedded C (STM32 HAL), XBee API communication
- MATLAB-based serial server for managing sensor transmissions and data
- Based on a hardware frontend capable of **rejecting drift noise** and enabling long-range (~2m) capacitive sensing
