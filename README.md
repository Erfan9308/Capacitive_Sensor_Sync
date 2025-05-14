# Capacitive Sensor Network Synchronization Project

This project implements a multi-sensor data acquisition system using STM32L4 microcontrollers and XBee modules. Each sensor node transmits synchronized readings based on ID-based time-sharing, coordinated by a MATLAB server over serial communication.

## Structure

- `keil_firmware/`: STM32 firmware (Keil uVision5 project)
- `matlab_server/`: MATLAB script to control timing and receive sensor data

## Highlights

- Real-time synchronized data collection from 4 capacitive sensors
- Embedded C (STM32 HAL), XBee API communication
- MATLAB-based serial server for managing sensor transmissions
