# ⚡ arduino_high_freq_pwm_pin11

![Hardware](https://img.shields.io/badge/Hardware-Arduino_Uno_ATmega328P-orange?style=for-the-badge&logo=arduino)
![Software](https://img.shields.io/badge/Software-AVR_C_Registers-blue?style=for-the-badge)
![Frequency](https://img.shields.io/badge/Max_Freq-2.6MHz+-red?style=for-the-badge)

## 📖 Overview
The **arduino_high_freq_pwm_pin11** is a high-performance PWM driver that bypasses the standard Arduino abstraction layers. By directly manipulating **Timer 2**, this module achieves frequencies and features impossible with standard `analogWrite()`.

The project supports two distinct operational modes:
1. **Continuous PWM Mode:** Generates a stable signal at a custom frequency and duty cycle.
2. **Pulse Counting Mode:** Generates a precise number of pulses (e.g., exactly 500 pulses) and then automatically shuts down via interrupts.

## 🚀 Key Features
* **Manual Timer Scaling:** Dynamically switches between prescalers (1, 8, 32, 64, 128, 256, 1024) to maintain the highest possible resolution for any given frequency.
* **CTC Mode Control:** Uses "Clear Timer on Compare Match" to define custom TOP values, allowing for non-standard frequencies (unlike Fast PWM which is often locked to powers of 2).
* **Burst Mode:** The `generatePWMPulses` function allows for "single-shot" burst generation, critical for medical instrumentation or precise mechanical movements.
* **High-Speed Toggle:** Capable of reaching frequencies up to ~2.6MHz (limited by the 16MHz system clock and ISR overhead).



## 🛠️ Technical Specifications
* **Pin Used:** Digital Pin 11 (OC2A).
* **Timer:** 8-bit Timer 2.
* **Frequency Range:** < 1Hz to ~2.6MHz.
* **Duty Cycle Resolution:** Varies with frequency (higher frequencies result in lower duty cycle resolution due to smaller TOP values).



## ⚙️ How it Works: The Logic
1. **The Frequency (OCR2A):** We define the "ceiling" of the timer. When the timer hits this value, it triggers `TIMER2_COMPA_vect`.
2. **The Duty Cycle (OCR2B):** We define a point inside that ceiling. When the timer hits this value, it triggers `TIMER2_COMPB_vect`.
3. **The Interrupts:** - At **Match B**, we turn the pin **LOW**.
   - At **Match A (TOP)**, we turn the pin **HIGH**. 
   - The time ratio between B and A creates the Duty Cycle.

## 👤 Author
* **Abdelrahman Elnahrawy**
* GitHub: [@Abdelrahman-Elnahrawy](https://github.com/Abdelrahman-Elnahrawy)

## ⚖️ License
This project is licensed under the **MIT License**.