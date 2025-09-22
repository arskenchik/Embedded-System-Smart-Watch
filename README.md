# Embedded-System-Smart-Watch
Smart Watch firmware on Arduino Nano 33 BLE Sense using nRF5 SDK and Segger Studio. Features: step counter (LSM accel + OLED), gesture control (APDS-9960 + LEDs), clock/logo modes, BLE CTS time sync, and power-saving (IMU duty cycle, OLED sleep). Built for accuracy, efficiency, and stability.
# Smart Watch on Nano BLE33

Embedded Systems Final Project (EE414)  
Team 16 – Abai Sansyzbai and collaborators  

---

## Overview
This project is a **Smart Watch firmware** running on the [Arduino Nano 33 BLE Sense](https://store.arduino.cc/products/arduino-nano-33-ble-sense), developed using **nRF5_SDK_17.1.0_ddde560** and **Segger Embedded Studio**.  
The watch integrates step counting, gesture recognition, an OLED user interface, and Bluetooth time synchronization.

---

## Features
- **Step Counter**
  - Adaptive threshold algorithm on LSM accelerometer data
  - Cross-checked against commercial smartwatch/manual counts
  - Displayed as `"step:XXXX"` on OLED

- **Gesture Control**
  - APDS-9960 gesture sensor
  - Maps swipe directions to:
    - **UP** → step display
    - **DOWN** → invert/reset OLED
    - **LEFT** → logo screen
    - **RIGHT** → clock screen
  - RGB LED feedback (RED/BLUE/GREEN/WHITE)

- **OLED Display**
  - Time (HH:MM:SS), steps, or logo
  - Custom bitmap fonts
  - Auto sleep after inactivity, wake on gesture

- **Bluetooth (BLE)**
  - Current Time Service (CTS) client
  - Syncs watch clock with central device
  - Advertising, bonding, and whitelist supported

- **Power Efficiency**
  - Accelerometer duty cycling (50 Hz active sampling, periodic sleep)
  - OLED auto-off after inactivity
  - Average current ~85 µA during trials:contentReference[oaicite:3]{index=3}

---

## Hardware
- **MCU**: Arduino Nano 33 BLE Sense (nRF52840 SoC)
- **Sensors**:
  - LSM Accelerometer (step counting)
  - APDS-9960 Gesture/Proximity sensor
- **Display**: SSD1306-based OLED (128×64)
- **Indicators**: RGB LEDs

---

## Software
- **SDK**: Nordic Semiconductor `nRF5_SDK_17.1.0_ddde560`
- **IDE**: Segger Embedded Studio
- **BLE Stack**: SoftDevice S140
- **Libraries**: CMSIS-DSP (for filtering), nrf_drv_* (TWI, timers, GPIO)

---

## Project Structure
