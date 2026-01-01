# LCD Keypad Stepper Controller

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Arduino IDE](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Display](https://img.shields.io/badge/LCD-Keypad%20Shield-lightgrey?style=for-the-badge)
![Motor](https://img.shields.io/badge/Motor-Stepper-blue?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Author](https://img.shields.io/badge/Author-Abdelrahman--Elnahrawy-orange?style=for-the-badge)

## Overview
User interface on an LCD Keypad Shield to control a stepper motor in speed or angle modes. A rotary encoder (A/B/Z) provides feedback via pin change interrupts, while the step pulses are generated from Timer2 OC2A on pin 11.

## Features
- Speed and Angle control modes with digit-by-digit editing.
- Direction display (CW/CC) and toggle.
- Encoder feedback for angle and RPM (uses Z index for RPM).
- Non-blocking UI loop; periodic control updates.

## Hardware
- Board: Arduino Uno/Nano
- LCD Keypad Shield: uses D8,D9,D4,D5,D6,D7 and A0 for buttons
- Encoder: A on A1, B on A2, Z on A3 (INPUT_PULLUP)
- Stepper driver: Enable=D13 (LOW=enable), DIR=D12, STEP=D11 (Timer2 OC2A)

### Wiring
- LCD keypad shield plugs onto the Arduino (A0 used for buttons).
- Encoder signals to A1/A2/A3 with GND and VCC (use debouncing or clean signals as needed).
- Stepper driver (e.g., DRV8825/A4988):
  - STEP -> D11
  - DIR  -> D12
  - EN   -> D13 (LOW=enable)
  - VMOT/GND per driver requirements

## Libraries / Dependencies
- LiquidCrystal (built-in)
- PinChangeInterrupt (via Library Manager)

## Build and Upload
1. Open `lcd_keypad_stepper_controller.ino`.
2. Select board and port.
3. Verify and Upload.

## Usage
- Use LEFT/RIGHT to move the cursor, UP/DOWN to change digits.
- SELECT toggles cursor lock and switches modes from the selection screen.
- Speed control sets a Timer2 frequency proportional to RPM.
- Angle control is a basic example; for precise moves, implement counted pulses with an ISR.

## License
MIT License

Copyright (c) 2025 Abdelrahman Elnahrawy

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Author
Abdelrahman Elnahrawy