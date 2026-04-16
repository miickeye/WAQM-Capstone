# WAQM – Wearable Air Quality Monitor

## Overview

The Wearable Air Quality Monitor (WAQM) is a portable system designed to measure environmental conditions in real time. The device monitors carbon monoxide (CO), oxygen (O₂), particulate matter (PM2.5/PM10), temperature, and humidity.

The system provides immediate visual and audible alerts when unsafe conditions are detected and supports wireless data transmission via Bluetooth Low Energy (BLE).

---

## System Components

- Microcontroller: ESP32-S3 DevKitC-1
- CO Sensor: Electrochemical (ME2-CO)
- O₂ Sensor: Electrochemical (ME2-O2)
- Environmental Sensor: SEN54 (PM, temperature, humidity)
- Signal Conditioning: Transimpedance amplifier (TIA)
- Communication: BLE
- Power: Li-ion battery with boost converter

---

## Features

- Continuous environmental monitoring
- Threshold-based alert system (LED and buzzer)
- BLE data transmission
- Portable and wearable design
- Designed for full-shift operation (8+ hours)

---

## Project Structure

---

## Setup

1. Install:
   - Visual Studio Code
   - PlatformIO extension

2. Clone the repository:

3. Build and upload:
- Connect ESP32-S3 via USB
- Open project in VS Code
- Use PlatformIO to build and upload firmware

---

## Operation

The system samples sensor data periodically. The microcontroller processes the data and compares values to predefined thresholds.

If a threshold is exceeded:
- LED indicators change state
- Buzzer activates
- Data is transmitted via BLE

---

## Testing and Calibration

- CO sensor calibrated using controlled gas exposure
- Threshold targets:
  - 35 ppm (warning)
  - 50 ppm (danger)
- Oxygen monitoring range:
  - Normal: ~20.9%
  - Warning: <19.5%
- PM levels based on EPA standards

---

## Project Status

In development. Current work includes:
- Sensor integration
- BLE communication
- Calibration and testing
- Mobile application interface

---

## Authors

Michael Woldai  
Tobin Mitchell  

California State University, Chico  
EECE 490 Capstone