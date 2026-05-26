# WAQM – Wearable Air Quality Monitor

## Overview

The Wearable Air Quality Monitor (WAQM) is a portable system designed to measure environmental conditions in real time. The device monitors carbon monoxide (CO), oxygen (O₂), particulate matter (PM2.5/PM10), temperature, and humidity.

The system provides immediate visual and audible alerts when unsafe conditions are detected and supports wireless data transmission via Bluetooth Low Energy (BLE).

---

## System Components

- Microcontroller: ESP32-S3 DevKitC-1
- CO Sensor: Electrochemical (ZE07-CO)
- O₂ Sensor: Electrochemical (ME2-O2)
- Environmental Sensor: SEN54 (PM, temperature, humidity)
- Signal Conditioning: Transimpedance amplifier (TIA)
- Communication: BLE, ADC, UART, I2C
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

## System Block Diagram
<img width="780" height="487" alt="image" src="https://github.com/user-attachments/assets/4944f10f-7f4f-4b71-9d1a-b7fb90f55720" />

## Hardware Setup
<img width="428" height="541" alt="image" src="https://github.com/user-attachments/assets/8aff9c71-0589-4807-b1b5-61bc91b8bb2d" />

## BLE Communication 
<img width="1182" height="638" alt="image" src="https://github.com/user-attachments/assets/67d2a2e9-b3d4-4b49-b290-a910f15ab324" />


## Testing and Calibration
<img width="423" height="492" alt="image" src="https://github.com/user-attachments/assets/a71325ce-6947-432d-bb1b-859c06a0c34a" />
<img width="1216" height="651" alt="image" src="https://github.com/user-attachments/assets/a6a5667c-b524-432f-8601-a61170eb0067" />
<img width="538" height="405" alt="image" src="https://github.com/user-attachments/assets/62744fb7-94a4-4eff-9478-e909ddf72746" />
<img width="1210" height="596" alt="image" src="https://github.com/user-attachments/assets/14acf776-529e-4b3b-b066-1b8490b0dde7" />
<img width="486" height="451" alt="image" src="https://github.com/user-attachments/assets/ad7452be-50c1-4714-b5c5-780676f8609f" />



## Authors

Michael Woldai  
Tobin Mitchell  

California State University, Chico  
EECE 490 Capstone
