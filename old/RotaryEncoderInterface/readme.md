# 🔄 Rotary Encoder Interface (Encoder_E)

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Arduino IDE](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Type](https://img.shields.io/badge/Input-Rotary%20Encoder-lightgrey?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Author](https://img.shields.io/badge/Author-Abdelrahman--Elnahrawy-orange?style=for-the-badge)

## 📖 Overview
The **Encoder_E** project provides a high-performance interface for **Incremental Rotary Encoders**. Unlike a standard potentiometer, a rotary encoder can spin infinitely and provides digital pulses that allow you to track both the **position** and **direction** of rotation.

This implementation uses **Hardware Interrupts** to ensure that every "click" of the encoder is captured, even if the main loop of the Arduino is busy with other tasks.

## 🚀 Key Features
* **🎯 Precision Tracking:** Accurate quadrature decoding to prevent missed pulses.
* **⚡ Interrupt-Driven:** Uses `attachInterrupt()` on pins D2 and D3 for real-time responsiveness.
* **🔄 Direction Detection:** Logic to determine Clockwise (CW) vs. Counter-Clockwise (CCW) movement.
* **🔘 Integrated Switch:** Support for the built-in push button found on common modules (KY-040).

## 🛠️ Hardware Requirements
* **Microcontroller:** Arduino Uno, Nano, or Mega.
* **Encoder Module:** KY-040 Rotary Encoder (or similar).
* **Components:** 0.1µF capacitors (optional, for hardware debouncing).

## 🔌 Pin Configuration
| Encoder Pin | Arduino Pin | Function |
| :--- | :--- | :--- |
| **CLK** | D2 (Interrupt) | Output A (Quadrature) |
| **DT** | D3 (Interrupt) | Output B (Quadrature) |
| **SW** | D4 | Internal Push Button |
| **VCC** | 5V | Power |
| **GND** | GND | Ground |



## ⚙️ How it Works
1. **Quadrature Signals:** The encoder outputs two square waves (CLK and DT) that are 90 degrees out of phase.
2. **Logic:** By checking the state of the DT pin at the moment the CLK pin changes state, the code determines the direction.
3. **Counter:** A global variable increments or decrements based on the rotation, which can be used to control menus, volume, or motor positions.

## 👤 Author
* **Abdelrahman Elnahrawy**
* GitHub: [@Abdelrahman-Elnahrawy](https://github.com/Abdelrahman-Elnahrawy)

## ⚖️ License
This project is licensed under the **MIT License**.
