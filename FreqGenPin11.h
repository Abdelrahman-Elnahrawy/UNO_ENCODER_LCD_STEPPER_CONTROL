#ifndef PWM_H
#define PWM_H

#include <Arduino.h>

#define OUTPUT_PIN 11
void FreqGenPin11Init();
void setFreqPin11(float frequency);
void DisableFreqGenPin11();
void EnableFreqGenPin11();
void FreqGenGeneratePulses(int numPulses, float frequency);
#endif