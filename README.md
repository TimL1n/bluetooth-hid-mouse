# Bluetooth HID Motion-Controlled Mouse

A Bluetooth HID mouse implemented on the ESP32-C3 using ESP-IDF and an ICM-42670-P IMU. The device allows a user to control a computer cursor by tilting the board and perform mouse clicks using the onboard BOOT button.

## Overview

This project combines Bluetooth Low Energy (BLE), HID device communication, I2C sensor interfacing, interrupt handling, and real-time embedded control to create a wireless motion-controlled mouse.

The ESP32-C3 reads acceleration data from an ICM-42670-P inertial measurement unit (IMU) over I2C, processes the sensor data to determine tilt direction and magnitude, and sends cursor movement commands to a paired computer through the BLE HID protocol.

## Features

* Bluetooth Low Energy HID mouse functionality
* Real-time cursor control using board tilt
* ICM-42670-P accelerometer integration over I2C
* Adjustable cursor speed based on tilt magnitude
* Time-based acceleration scaling for smoother movement
* Deadzone filtering to eliminate unintended cursor drift
* BOOT button support for left mouse click
* Wireless operation with standard HID compatibility

## Hardware

* ESP32-C3 Development Board
* ICM-42670-P IMU
* USB power source

## Software

* ESP-IDF
* Bluetooth Low Energy (BLE)
* HID Device Profile
* FreeRTOS
* C Programming Language

## System Architecture

1. The ESP32-C3 initializes the BLE HID profile and establishes a Bluetooth connection.
2. The ICM-42670-P accelerometer is configured through the I2C interface.
3. Acceleration values are continuously sampled at a fixed update rate.
4. Tilt magnitude and direction are calculated from the sensor data.
5. Deadzone filtering removes small unintentional movements.
6. Cursor velocity is determined based on tilt thresholds and acceleration scaling.
7. Mouse movement reports are transmitted to the host computer using BLE HID.
8. The onboard BOOT button generates left-click events.

## Motion Control Algorithm

The cursor movement system uses multiple processing stages:

### Deadzone Filtering

Small accelerometer values are ignored to prevent cursor drift caused by sensor noise and hand jitter.

### Speed Levels

Two movement speeds are used:

* Small tilt → Slow cursor movement
* Large tilt → Faster cursor movement

### Dynamic Acceleration

If the board remains tilted in the same direction for an extended period, the cursor speed gradually increases. This allows precise control for small adjustments while maintaining efficient movement across larger screen distances.

## Challenges

### BLE HID Integration

Configuring the ESP32-C3 as a Bluetooth HID device required understanding HID report structures and BLE communication workflows.

### Sensor Noise

Raw accelerometer measurements contained small fluctuations that caused unintended cursor movement. Deadzone filtering and threshold tuning were implemented to improve stability.

### Cursor Responsiveness

Finding a balance between responsiveness and smooth movement required extensive testing of update rates, tilt thresholds, and acceleration scaling factors.

## Results

The final system successfully functions as a wireless Bluetooth mouse compatible with standard desktop operating systems. Cursor movement responds to board tilt with smooth acceleration behavior, and the BOOT button provides reliable click functionality.

## Skills Demonstrated

* Embedded C Programming
* ESP-IDF Development
* Bluetooth Low Energy (BLE)
* HID Device Profiles
* I2C Communication
* IMU Sensor Integration
* Interrupt Handling
* Real-Time Embedded Systems
* FreeRTOS
* Embedded Debugging

## Future Improvements

* Gyroscope integration for improved motion tracking
* Adjustable sensitivity settings
* Gesture recognition
* Additional mouse buttons
* Battery-powered operation
* Wireless configuration interface

## Repository Structure

```text
.
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
└── main/
    ├── main.c
    ├── hid_device_le_prf.c
    ├── hid_dev.c
    ├── hid_dev.h
    ├── esp_hidd_prf_api.c
    ├── esp_hidd_prf_api.h
    └── hidd_le_prf_int.h
```

## Acknowledgements

This project was developed using the ESP-IDF framework and BLE HID libraries provided by Espressif. Custom application logic, motion control algorithms, sensor integration, and cursor control functionality were developed for this project.
