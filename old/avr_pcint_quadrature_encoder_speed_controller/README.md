# avr_pcint_quadrature_encoder_speed_controller

![Platform](https://img.shields.io/badge/Platform-AVR_ATmega328P-blue?style=for-the-badge&logo=arduino)
![Type](https://img.shields.io/badge/Category-Embedded_Control-orange?style=for-the-badge)
![Status](https://img.shields.io/badge/Performance-High--Resolution_Timing-green?style=for-the-badge)

## 📖 Overview
The **avr_pcint_quadrature_encoder_speed_controller** is a high-precision firmware designed for tracking high-speed rotary encoders using **Pin Change Interrupts (PCINT)**. By leveraging the AVR **Timer2** hardware and extended 32-bit timestamping, this controller calculates angular position and RPM with microsecond resolution, making it suitable for closed-loop motor control and robotic odometry.

---

## 🚀 Key Technical Features

* **⏱️ Microsecond Precision:** Utilizes the 8-bit hardware Timer2 with a prescaler of 8 (2MHz) to achieve a resolution of 0.5μs per tick. 
* **🧬 Extended 32-bit Timestamping:** Implements an Interrupt Service Routine (ISR) for Timer2 overflows, effectively turning an 8-bit timer into a 32-bit high-resolution clock to prevent overflow errors during slow rotations.
* **⚡ Interrupt-Driven Architecture:** Uses the `PinChangeInterrupt` library to ensure that every pulse on Phase A, Phase B, and the Index (Z) channel is captured instantly, minimizing "dropped pulses" at high RPMs.
* **📏 Real-Time Metrics:**
    * **RPM Calculation:** Derived from the period between Index (Z) pulses for maximum accuracy.
    * **Angular Mapping:** Converts raw pulse counts into degrees ($0^\circ - 360^\circ$) based on the defined Pulses Per Revolution (PPR).
    * **Directional Logic:** Real-time comparison of Phase A vs. Phase B pulse frequency to determine rotation polarity.

---

## 🛠️ Hardware Architecture

### Encoder Timing Diagram


### Pin Assignment (Arduino Uno/Nano)
| Signal | Pin | Type | Function |
| :--- | :--- | :--- | :--- |
| **Phase A** | GPIO 8 | PCINT0 | Primary pulse counting |
| **Phase B** | GPIO 9 | PCINT1 | Secondary pulse counting / Direction |
| **Index Z** | GPIO 10 | PCINT2 | Absolute zero reference / RPM calc |

---

## 📊 Logic & Math
The system uses an **Atomic Reading** method to fetch the timer value. This ensures that if a timer overflow occurs exactly during a read, the 32-bit timestamp remains synchronized.

### RPM Calculation Formula
$$RPM = \frac{60 \times 1,000,000}{\text{Time in microseconds per revolution}}$$

### Angle Calculation Formula
$$\text{Angle} = \left( \frac{\text{Average Pulse Count}}{\text{PPR}} \right) \times 360$$

---

## 📂 Folder Structure
```text
avr_pcint_quadrature_encoder_speed_controller/
├── avr_pcint_quadrature_encoder_speed_controller.ino
├── encoder_logic_flow.png
└── README.md