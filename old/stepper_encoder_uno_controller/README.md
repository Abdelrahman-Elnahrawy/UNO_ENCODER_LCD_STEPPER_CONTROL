# Stepper + Encoder Uno Controller

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Arduino IDE](https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Display](https://img.shields.io/badge/LCD-Keypad%20Shield-lightgrey?style=for-the-badge)
![Motor](https://img.shields.io/badge/Motor-Stepper-blue?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Author](https://img.shields.io/badge/Author-Abdelrahman--Elnahrawy-orange?style=for-the-badge)

## Overview
A controller for a stepper motor with rotary encoder feedback on Arduino Uno/Nano. Includes an LCD Keypad Shield UI for speed/angle commands, counted pulses for precise angle moves, and cleaned code with well-documented timers and interrupts.

## Features
- Speed and Angle modes with digit editing.
- Encoder feedback via A/B/Z (PinChangeInterrupt) and Timer1.
- Timer2 OC2A (Pin 11) for step pulse generation; counted pulses for moves.
- Initial homing-style sequence then interactive control.

## Hardware
- Board: Arduino Uno/Nano
- LCD Keypad Shield: D8,D9,D4,D5,D6,D7 + A0 for buttons
- Encoder: A1 (A), A2 (B), A3 (Z)
- Stepper driver: STEP=D11 (OC2A), DIR=D12, EN=D13 (LOW=enable)

## Libraries / Dependencies
- LiquidCrystal (built-in)
- PinChangeInterrupt (Library Manager)

## Build and Upload
1. Open `stepper_encoder_uno_controller.ino`.
2. Select board and port.
3. Verify and Upload.

## Usage
- On first startup, the sketch performs a full revolution and nudges toward 0 degrees.
- Use the LCD keypad to select Speed or Angle mode and adjust values.
- SELECT toggles cursor lock and transitions out of selection mode.

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