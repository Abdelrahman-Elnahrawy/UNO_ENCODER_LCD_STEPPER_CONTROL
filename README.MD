# Stepper & Encoder Closed-Loop Control System

![Arduino](https://img.shields.io/badge/Platform-Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Language](https://img.shields.io/badge/Language-C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Stable-green?style=for-the-badge)

A high-precision firmware for Arduino Uno designed to control a Stepper Motor using a Rotary Encoder for closed-loop feedback. This system features PID-regulated speed control, absolute angle positioning, and a real-time LCD interface.

---

## 🚀 Key Features

* **Closed-Loop Speed Control:** Utilizes PID (Proportional-Integral-Derivative) logic to maintain consistent RPM under varying loads.
* **Precise Positioning:** Target specific angles with high accuracy using encoder feedback.
* **Interactive GUI:** 16x2 LCD support with a 5-button keypad for real-time mode switching and parameter adjustment.
* **Advanced Timing:** Uses hardware timers and interrupts for high-frequency encoder tracking and pulse generation.
* **Multiple Modes:** Easily switch between `Speed Control`, `Angle Control`, and `Selection` modes.

---

## 🛠 Hardware Configuration

The system is optimized for the **Arduino Uno** architecture.

| Component | Pin (Uno) | Description |
| :--- | :--- | :--- |
| **Stepper Step** | Pin 11 | PWM/Frequency Output (Timer2) |
| **Stepper Dir** | Pin 12 | Direction Logic |
| **Stepper Enable** | Pin 13 | Motor Driver Toggle |
| **Encoder A** | Pin A1 | Phase A Input |
| **Encoder B** | Pin A2 | Phase B Input |
| **Encoder Z** | Pin A3 | Index/Home Pulse |
| **LCD Pins** | 8, 9, 4, 5, 6, 7 | RS, EN, D4, D5, D6, D7 |
| **Keypad** | Pin A0 | Analog Input for Buttons |

---

## ⚙️ PID Tuning

The PID constants can be tuned within the main `.ino` file to match your specific motor and load requirements:

```cpp
float rpmKp = 0.20;  // Proportional Gain
float rpmKi = 0.05;  // Integral Gain
float rpmKd = 0.32;  // Derivative Gain
```
#📂 Project Structure
Encoder.cpp / .h: High-speed encoder pulse tracking and RPM calculation.

Stepper_Handler.cpp / .h: Frequency-based stepper movement logic.

GUI_Handler.cpp / .h: LCD menu navigation and button debouncing.

FreqGenPin11.cpp / .h: Direct register manipulation for Timer2 pulse generation.

#📜 License
MIT License
Copyright (c) 2023

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.