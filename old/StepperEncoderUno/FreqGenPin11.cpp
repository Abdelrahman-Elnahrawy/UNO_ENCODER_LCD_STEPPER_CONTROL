/**
 * @file PWMPin11.h
 * @brief PWM Control for Pin 11 on Arduino Uno
 * 
 * This module provides functions to initialize and control PWM (Pulse Width Modulation) on Pin 11 
 * (OC2A) of the Arduino Uno. The PWM signal can be configured for different frequencies and duty cycles.
 * 
 * The module works by configuring Timer 2 in CTC (Clear Timer on Compare Match) mode, allowing precise 
 * control over the PWM signal characteristics. The frequency and duty cycle can be adjusted dynamically 
 * based on application requirements.
 * 
 * Features:
 * - Initialize PWM on Pin 11.
 * - Set frequency and duty cycle for the PWM signal.
 * - Enable and disable PWM output.
 * 
 * Connections:
 * - Pin 11: Connected to the output device for PWM control.
 * 
 * Usage:
 * - Call `PWMPin11Init()` to initialize PWM on Pin 11.
 * - Use `setPWMPin11(frequency, dutyCycle)` to set the desired frequency (in Hz) and duty cycle (0 to 100%).
 * - Call `enablePWMPin11()` to start the PWM output.
 * - Call `disablePWMPin11()` to stop the PWM output.
 */


#include "FreqGenPin11.h"


uint16_t TargetPulsesCount ;
uint16_t CurrentPulsesCount ;
uint8_t prescalerBits = 0;
// Function to initialize PWM on Pin 11
void FreqGenPin11Init() {
    pinMode(11, OUTPUT);  // Set Pin 11 as an output pin

    // Set Timer2 in CTC mode and stop it
    TCCR2A = 0;
    TCCR2B = 0;

    // Set CTC mode with OCR2A as TOP
    TCCR2A |= (1 << WGM21);  // Configure Timer2 for CTC mode
    TCCR2A |= (1 << COM2A0); // Toggle OC2A on compare match     
    TCCR2A |= (1 << COM2B0); // Toggle OC2B (Pin 3) on compare match with OCR2B
}

// Function to set PWM frequency and duty cycle
void setFreqPin11(float frequency) {
    // Calculate the required TOP value for the desired frequency
    unsigned long topValue;
    if(frequency == 0) {return;}
    // Find the smallest prescaler that will fit the desired frequency
    if (frequency >= 31373) {
        // Prescaler = 1
        prescalerBits = (1 << CS20);
        topValue = (16000000 / (1 * 2 * frequency)) - 1;
    } else if (frequency >= 3921) {
        // Prescaler = 8
        prescalerBits = (1 << CS21);
        topValue = (16000000 / (8 * 2 * frequency)) - 1;
    } else if (frequency >= 980) {
        // Prescaler = 32
        prescalerBits = (1 << CS21) | (1 << CS20);
        topValue = (16000000 / (32 * 2 * frequency)) - 1;
    } else if (frequency >= 490) {
        // Prescaler = 64
        prescalerBits = (1 << CS22);
        topValue = (16000000 / (64 * 2 * frequency)) - 1;
    } else if (frequency >= 245) {
        // Prescaler = 128
        prescalerBits = (1 << CS22) | (1 << CS20);
        topValue = (16000000 / (128 * 2 * frequency)) - 1;
    } else if (frequency >= 122) {
        // Prescaler = 256
        prescalerBits = (1 << CS22) | (1 << CS21);
        topValue = (16000000 / (256 * 2 * frequency)) - 1;
    } else {
        // Prescaler = 1024
        prescalerBits = (1 << CS22) | (1 << CS21) | (1 << CS20);
        topValue = (16000000 / (1024 * 2 * frequency)) - 1;
    }

    // Limit the topValue to a maximum of 255 (8-bit timer)
    if (topValue > 255) {
        topValue = 255;
    }

    // Set the TOP value and compare value
    OCR2A = topValue;       // Set TOP for frequency

    // Set the Timer2 prescaler
    TCCR2B = prescalerBits;

    // Configure the timer in CTC mode with toggle on compare match
    TCCR2A = (1 << WGM21) | (1 << COM2A0);  // CTC mode, toggle OC2A on compare
}

// Function to enable PWM output
void EnableFreqGenPin11() {
    TCCR2B = prescalerBits; // Start Timer2 with prescaler of the previous configuration
}

// Function to disable PWM output
void DisableFreqGenPin11() {
  TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20)); // Stop Timer2 by clearing all clock select bits
}

void FreqGenGeneratePulses(int numPulses, float frequency) {
  if(frequency == 0  || numPulses == 0 ) return;
   DisableFreqGenPin11();
    CurrentPulsesCount = 0;
    TargetPulsesCount = 2 * numPulses;
    // Enable Output Compare Match A Interrupt
    TIMSK2 |= (1 << OCIE2A); // Enable interrupt on OCR2A match
    setFreqPin11(frequency);
    EnableFreqGenPin11();
}

ISR(TIMER2_COMPA_vect) {
  CurrentPulsesCount++;
  if(CurrentPulsesCount>=TargetPulsesCount){
        TIMSK2 &= ~(1 << OCIE2A); // Disable interrupt on OCR2A match
        DisableFreqGenPin11();
        CurrentPulsesCount = 0;
  }

}
