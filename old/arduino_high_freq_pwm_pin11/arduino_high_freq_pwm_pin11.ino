/**
 * Project: High-Frequency Manual PWM Controller
 * Hardware: Arduino Uno (ATmega328P)
 * Description: Uses Timer 2 in CTC mode to generate precise PWM frequencies 
 * on Pin 11, including a pulse-counting mode for stepper or burst control.
 */

#include <Arduino.h>

// --- Global State Variables ---
volatile uint16_t TargetPulsesCount = 0;
volatile uint16_t CurrentPulsesCount = 0;
uint8_t prescalerBits = 0;

// Function Pointer for dynamic Interrupt behavior
void (*OnCompareAInterruptFun)();

// --- Forward Declarations ---
void OncompareAinterruptCasePWM();
void OncompareAinterruptCasePulses();
void DisablePWMPin11();

/**
 * ISR for Case 1: Simple PWM Mode
 * Toggles pin state to maintain frequency/duty cycle.
 */
void OncompareAinterruptCasePWM() {
    digitalWrite(11, HIGH); 
}

/**
 * ISR for Case 2: Pulse Counting Mode
 * Stops the timer once the target number of pulses is reached.
 */
void OncompareAinterruptCasePulses() {
    digitalWrite(11, HIGH); 
    CurrentPulsesCount++;
    if (CurrentPulsesCount >= TargetPulsesCount) {
        TIMSK2 &= ~(1 << OCIE2A); // Disable A interrupt
        DisablePWMPin11();
        CurrentPulsesCount = 0; 
    }
}

/**
 * Configures Timer 2 for CTC (Clear Timer on Compare Match) Mode.
 */
void PWMPin11Init() {
    pinMode(11, OUTPUT);

    TCCR2A = 0;
    TCCR2B = 0;

    // WGM21 = CTC Mode
    TCCR2A |= (1 << WGM21);  
    
    // Enable interrupts for both A (Top/Frequency) and B (Duty Cycle)
    TIMSK2 |= (1 << OCIE2B) | (1 << OCIE2A);
    
    // Default to standard PWM behavior
    OnCompareAInterruptFun = &OncompareAinterruptCasePWM;
    sei(); // Ensure global interrupts are enabled
}

/**
 * Calculates frequency TOP and duty cycle OCR values based on 16MHz Clock.
 */
void setPWMPin11(float frequency, float dutyCycle) {
    if (frequency <= 0) return;

    unsigned long topValue;
    float baseClock = 16000000.0;

    // Prescaler Selection Logic
    if (frequency >= 62500) {
        prescalerBits = (1 << CS20); // /1
        topValue = (baseClock / (1.0 * frequency)) - 1;
    } else if (frequency >= 7813) {
        prescalerBits = (1 << CS21); // /8
        topValue = (baseClock / (8.0 * frequency)) - 1;
    } else if (frequency >= 977) {
        prescalerBits = (1 << CS22); // /64
        topValue = (baseClock / (64.0 * frequency)) - 1;
    } else {
        prescalerBits = (1 << CS22) | (1 << CS21) | (1 << CS20); // /1024
        topValue = (baseClock / (1024.0 * frequency)) - 1;
    }

    OCR2A = (topValue > 255) ? 255 : (uint8_t)topValue;
    OCR2B = (uint8_t)((dutyCycle * (OCR2A + 1)) / 100.0);

    TCCR2B = prescalerBits; // Apply clock
}

void DisablePWMPin11() {
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
    digitalWrite(11, LOW);
}

void EnablePWMPin11() {
    TCCR2B |= prescalerBits;
}

/**
 * Generates a specific number of pulses then stops.
 * Useful for precise dosing or stepper step generation.
 */
void generatePWMPulses(int numPulses, float frequency, float dutyCycle) {
    if (frequency <= 0 || dutyCycle <= 0) return;
    DisablePWMPin11();
    CurrentPulsesCount = 0;
    TargetPulsesCount = numPulses;
    OnCompareAInterruptFun = &OncompareAinterruptCasePulses;
    setPWMPin11(frequency, dutyCycle);
    TIMSK2 |= (1 << OCIE2A); 
    EnablePWMPin11();
}

// --- Interrupt Service Routines ---

ISR(TIMER2_COMPA_vect) {
    (*OnCompareAInterruptFun)();
}

ISR(TIMER2_COMPB_vect) {
    digitalWrite(11, LOW); 
}

void setup() {
    Serial.begin(115200);
    PWMPin11Init();
}

void loop() {
    // Sweep through frequencies from 30Hz to 2.6MHz
    float testFreqs[] = {30, 500, 1000, 10000, 50000, 100000, 500000, 2666666};
    for (float f : testFreqs) {
        Serial.print("Testing Freq: "); Serial.println(f);
        setPWMPin11(f, 50);
        delay(2000);
    }
}