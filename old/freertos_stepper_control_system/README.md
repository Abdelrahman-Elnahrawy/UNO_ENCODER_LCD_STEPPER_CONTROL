# ⚙️ freertos_stepper_control_system

![RTOS](https://img.shields.io/badge/System-FreeRTOS-blue?style=for-the-badge)
![Hardware](https://img.shields.io/badge/Hardware-Arduino_Uno-orange?style=for-the-badge)
![Motor](https://img.shields.io/badge/Actuator-Stepper_Motor-green?style=for-the-badge)

## 📖 Overview
This project represents a shift from simple sequential programming to **Concurrent Embedded Systems**. By implementing **FreeRTOS**, the system manages three distinct operations simultaneously:
1. **User Interface:** Handling LCD menus and button debouncing.
2. **Motion Control:** Generating precise step pulses for a motor.
3. **Input Sensing:** Monitoring a rotary encoder for parameter adjustment.

## 🚀 Key RTOS Concepts
* **Multitasking:** Each function runs in its own "Task" with its own stack memory.
* **Prioritization:** The Stepper Task is given higher priority to ensure smooth motor movement without "jitter" caused by LCD updates.
* **Deterministic Timing:** `vTaskDelay` ensures that tasks run at exact intervals, unlike the standard `delay()` which blocks the entire CPU.



[Image of stepper motor driver A4988 wiring diagram]


## 🛠️ Menu Structure
* **Mode 0:** Selection screen (choose between Speed and Angle).
* **Mode 1 (Speed):** Adjust motor RPM in real-time.
* **Mode 2 (Angle):** Rotate motor to a specific degree.

## 📐 Hardware Architecture
| Component | Pin | Purpose |
| :--- | :--- | :--- |
| **STEP** | GPIO 3 | Pulse generation |
| **DIR** | GPIO 4 | Direction control |
| **ENABLE** | GPIO 5 | Driver power control |
| **LCD RS/E/D4-D7**| 8, 9, 4, 5, 6, 7 | UI Display |
| **Buttons** | Analog 0 | Menu Navigation |

## 👤 Author
* **Abdelrahman Elnahrawy**
* GitHub: [@Abdelrahman-Elnahrawy](https://github.com/Abdelrahman-Elnahrawy)

## ⚖️ License
This project is licensed under the **MIT License**.