# LCD Keypad Stepper Closed-Loop Controller

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Arduino IDE](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Display](https://img.shields.io/badge/LCD-Keypad%20Shield-lightgrey?style=for-the-badge)
![Control](https://img.shields.io/badge/Control-Closed--Loop-blue?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Author](https://img.shields.io/badge/Author-Abdelrahman--Elnahrawy-orange?style=for-the-badge)

## Overview
Closed-loop stepper controller using an LCD Keypad Shield UI and a rotary encoder (A/B/Z) for feedback. Supports speed and angle modes. Step pulses are generated using Timer2 OC2A (Pin 11); angle moves use counted pulses, and speed mode can apply simple PID correction.

## Features
- LCD Keypad UI with mode selection and digit editing.
- Encoder feedback for angle and RPM (via Z index and Timer1 timebase).
- Counted pulses for angle moves; free-run for speed moves.
- Parameterized encoder wraps using ENCODER_PPR.
- Timer2 toggles only OC2A (Pin 11).

## Hardware
- Board: Arduino Uno/Nano
- LCD Keypad Shield: D8,D9,D4,D5,D6,D7 and A0
- Encoder: A1 (A), A2 (B), A3 (Z) with INPUT_PULLUP
- Stepper driver: STEP=D11 (OC2A), DIR=D12, EN=D13 (LOW=enable)

## Libraries / Dependencies
- LiquidCrystal (built-in)
- PinChangeInterrupt (Library Manager)

## Build and Upload
1. Open `lcd_keypad_stepper_closed_loop.ino`.
2. Select board and port.
3. Verify and Upload.

## Usage
- Use keyboard buttons to set Speed or Angle. SELECT toggles cursor lock and mode transition.
- Angle mode: periodic corrections generate counted pulses towards the target angle.
- Speed mode: periodic correction uses simple PID to adjust commanded RPM.

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