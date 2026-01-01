# Gesture Controlled Car using STM32 and LoRa

This project demonstrates a **long-range gesture-controlled robotic car**
using **STM32 Blue Pill**, **MPU6050 IMU**, and **SX1278 LoRa** modules.

Hand movements are captured using an MPU6050 sensor and transmitted wirelessly
via LoRa to control a robotic car in real time.

## 🚀 Features
- Gesture-based control (roll & pitch)
- Long-range communication using SX1278 LoRa
- Real-time motor control
- Low-power embedded design
- Fully working hardware prototype

## 🧠 Technologies Used
- STM32F103C8 (Blue Pill)
- MPU6050 (I2C)
- SX1278 LoRa (SPI)
- STM32CubeIDE & STM32CubeMX
- DC/DC converters
- Li-ion battery power system

## 🛠 System Overview

### Transmitter (Gesture Controller)
- STM32 Blue Pill
- MPU6050 IMU
- SX1278 LoRa
- Powered by 3× Li-ion batteries + DC/DC converter

### Receiver (Car)
- STM32 Blue Pill
- SX1278 LoRa
- Dual DC motors
- H-Bridge motor driver
- Powered by 2× Li-ion batteries + DC/DC converter

## 📌 Notes
- Circuit schematics are not included yet.
- All configurations are available via STM32CubeMX (.ioc files).
- Code is well-structured and ready for extension.

## 📫 Contact
If you are interested in this project or have suggestions:
- LinkedIn: www.linkedin.com/in/edvard-grigoryan-b260ab2a0
