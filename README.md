# RPI_2040-flight-controller

This project is a custom-built quadcopter flight controller developed on the Raspberry Pi Pico (RP2040) using an MPU6050 IMU and hardware PWM at 400Hz for ESC control. The aim of this project is to gain a deep, low-level understanding of multirotor stabilisation, control theory, and embedded flight software

## Current Features

- Raspberry Pi Pico (RP2040) based flight controller
- MPU6050 IMU (gyroscope + accelerometer)
- Complementary filter for angle estimation
- Rate (Acro) mode for manual flight
- Angle (Self-Level) mode using an outer P loop
- 400Hz hardware PWM for ESCs
- Quad-X motor mixing
- Digital filtering (PT1 low-pass filtering)
- ESC calibration mode
- Modular code structure for easy tuning
- Designed for DIY FPV and experimental aircraft

---

## What This Project Focuses On

This project is built as a **learning and experimentation platform** for:

- Understanding PID control in multirotor systems
- Studying IMU noise, drift, and filtering
- Exploring motor mixing algorithms
- Optimising loop timing and real-time performance
- Building a flight controller from the ground up

Rather than aiming to be a polished consumer product, the goal is to **learn by building everything from scratch**.

---

## BOM

- Raspberry Pi Pico (RP2040)
- MPU6050 (I2C)
- Brushless DC motors
- SimonK / BLHeli ESCs
- Li-Po battery
- RC receiver / ESP32 transmitter (optional)
- frame
- NRF24L01 module

<p align="center">
  <img src="Drone_5/Images/Drone 1.jpeg" width="500">
</p>

# Transmitter

## ✨ Current Features

- ESP32-based drone transmitter and control unit
- Reads multiple analogue inputs (joysticks / potentiometers)
- Supports additional inputs (toggle switches and push buttons)
- wireless communication to the flight controller
- Stick calibration procedure
- Battery monitoring
- Expandable design for telemetry and configuration features
- Built using the Arduino framework for rapid development
- Designed to integrate with DIY drone and robotics projects

---

## 🧭 BOM

- ESP32 development board
- 2x Dual-axis joystick modules
- 2x Potentiometers
- 2x Toggle switches
- Push buttons
- NRF24L01 module
- 3.7V Li-ion battery
- Custom 3D-printed transmitter enclosure
- I2C 16x2 LCD display
- Custom PCB

<p align="center">
  <img src="Transmitter_2/Images/Transmitter 1.jpeg" width="240">
  <img src="Transmitter_2/Images/Transmitter 2.jpeg" width="240">
</p>

<p align="center">
  <img src="Transmitter_2/Images/Transmitter 3.jpeg" width="240">
  <img src="Transmitter_2/Images/Transmitter 4.jpeg" width="240">
</p>

<p align="center">
  <img src="Transmitter_2/Images/Transmitter 5.jpeg" width="240">
</p>
