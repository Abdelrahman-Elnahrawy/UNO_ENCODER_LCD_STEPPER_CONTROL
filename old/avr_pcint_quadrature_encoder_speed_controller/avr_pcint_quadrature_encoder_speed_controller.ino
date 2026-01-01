/**
 * PROJECT: AVR PCINT Quadrature Encoder Controller
 * AUTHOR: Abdelrahman Elnahrawy
 * DESCRIPTION: High-precision encoder tracking using Pin Change Interrupts 
 * and Timer2 overflow for RPM and Angular position calculation.
 */

#include "PinChangeInterrupt.h"

// --- Hardware Configuration ---
#define ENCODER_PINA  8   // Phase A
#define ENCODER_PINB  9   // Phase B
#define ENCODER_PINZ  10  // Index Pulse (Z)
#define ENCODER_PPR   300 // Pulses per revolution

#define CLOCKWISE        0
#define COUNTERCLOCKWISE 1

// --- Global Volatile Variables ---
volatile uint16_t EncoderPhaseACounter = 0;
volatile uint16_t EncoderPhaseBCounter = 0;
volatile uint32_t pulseTicksA = 0;
volatile uint32_t pulseTicksB = 0;
volatile uint32_t pulseTicksZ = 0;

volatile uint32_t lastTicksA = 0;
volatile uint32_t lastTicksB = 0;
volatile uint32_t lastTicksZ = 0;

volatile uint32_t timerOverflows = 0; // 32-bit to prevent easy overflow

// --- Conversion Macros ---
// Prescaler 8 on 16MHz clock = 2MHz (0.5us per tick)
#define TICKS_TO_US(ticks) ((ticks) / 2)
#define TICKS_TO_MS(ticks) ((ticks) / 2000.0)

// ================================================================
// INTERRUPT SERVICE ROUTINES (ISR)
// ================================================================

/**
 * Timer2 Overflow Vector: Triggers every 256 ticks.
 * Used to extend the 8-bit timer into a 32-bit timestamp.
 */
ISR(TIMER2_OVF_vect) {
  timerOverflows++;
}

inline uint32_t getFullTicks() {
  uint8_t low = TCNT2;
  uint32_t high = timerOverflows;
  
  // Handle edge case where timer overflows just before reading
  if ((TIFR2 & _BV(TOV2)) && (low < 128)) high++;
  return (high << 8) | low;
}

void EncoderPhaseA() {
  uint32_t currentTicks = getFullTicks();
  pulseTicksA = currentTicks - lastTicksA;
  lastTicksA = currentTicks;
  EncoderPhaseACounter++;
}

void EncoderPhaseB() {
  uint32_t currentTicks = getFullTicks();
  pulseTicksB = currentTicks - lastTicksB;
  lastTicksB = currentTicks;
  EncoderPhaseBCounter++;
}

void EncoderPhaseZ() {
  uint32_t currentTicks = getFullTicks();
  pulseTicksZ = currentTicks - lastTicksZ;
  lastTicksZ = currentTicks;
  
  // Sync point: Reset counters on Z-pulse
  EncoderPhaseACounter = 0; 
  EncoderPhaseBCounter = 0;
  // Note: We don't reset the timer here to keep a continuous timeline
}

// ================================================================
// API FUNCTIONS
// ================================================================

void EncoderInit() {
  pinMode(ENCODER_PINA, INPUT_PULLUP);
  pinMode(ENCODER_PINB, INPUT_PULLUP);
  pinMode(ENCODER_PINZ, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(ENCODER_PINA), EncoderPhaseA, RISING);
  attachPCINT(digitalPinToPCINT(ENCODER_PINB), EncoderPhaseB, RISING);
  attachPCINT(digitalPinToPCINT(ENCODER_PINZ), EncoderPhaseZ, RISING);

  // Setup Timer 2: Normal Mode, Prescaler 8
  TCCR2A = 0;
  TCCR2B = (1 << CS21); 
  TCNT2 = 0;
  TIMSK2 = (1 << TOIE2); // Enable Overflow Interrupt
}

float EncoderGetRPM() {
  if (pulseTicksZ == 0) return 0;
  // RPM = (60 * 1,000,000 us) / Time for one full Z-to-Z revolution
  return 60000000.0 / TICKS_TO_US(pulseTicksZ);
}

float EncoderGetAngle() {
  float avgCount = (EncoderPhaseACounter + EncoderPhaseBCounter) / 2.0;
  return avgCount * (360.0 / ENCODER_PPR);
}

bool EncoderGetDirection() {
  // Simplistic check based on pulse frequency
  return (EncoderPhaseACounter > EncoderPhaseBCounter) ? CLOCKWISE : COUNTERCLOCKWISE;
}

// ================================================================
// MAIN EXECUTION
// ================================================================

void setup() {
  Serial.begin(115200);
  EncoderInit();
  Serial.println("System Initialized: Encoder Node Online");
}

void loop() {
  Serial.print("Angle: "); Serial.print(EncoderGetAngle());
  Serial.print(" | RPM: "); Serial.println(EncoderGetRPM());
  delay(500);
}