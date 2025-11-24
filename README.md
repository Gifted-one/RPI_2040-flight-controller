# RPI_2040-flight-controller

This project is a custom-built quadcopter flight controller developed on the Raspberry Pi Pico (RP2040) using an MPU6050 IMU and hardware PWM at 400Hz for ESC control. The aim of this project is to gain a deep, low-level understanding of multirotor stabilisation, control theory, and embedded flight software

## ✨ Current Features

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

## 🔬 What This Project Focuses On

This project is built as a **learning and experimentation platform** for:

- Understanding PID control in multirotor systems
- Studying IMU noise, drift, and filtering
- Exploring motor mixing algorithms
- Optimising loop timing and real-time performance
- Building a flight controller from the ground up

Rather than aiming to be a polished consumer product, the goal is to **learn by building everything from scratch**.

---

## 🧭 Hardware Used

- Raspberry Pi Pico (RP2040)
- MPU6050 (I2C)
- Brushless DC motors
- SimonK / BLHeli ESCs
- Li-Po battery
- RC receiver / ESP32 transmitter (optional)
- 3D-printed or DIY frame
