
/**
 * @file Encoder.cpp
 * @brief This file contains the implementation for handling rotary encoder inputs.
 *
 * A rotary encoder is used to convert the angular position or motion of a shaft to 
 * an analog or digital signal. This implementation is specifically designed to work 
 * with a rotary encoder that produces two phase signals (A and B) to determine 
 * the direction of rotation as well as an additional signal (Z) for indexing or 
 * reset purposes. The encoder's counts, pulse durations, and rotation direction 
 * are calculated and can be accessed via functions.
 *
 * Key Features:
 * - Tracks the counts of the encoder phases (A and B)
 * - Measures pulse ticks between changes to calculate speed
 * - Determines the direction of rotation
 * - Resets counters and tick measurements on index signal (Z)
 * - Handles timer overflows using Timer2 to manage pulse timing
 *
 * Variables:
 * - EncoderPhaseACounter: Counts the number of phase A transitions
 * - EncoderPhaseBCounter: Counts the number of phase B transitions
 * - pulseTicksA, pulseTicksB, pulseTicksZ: Duration of pulses between transitions
 * - TimerOverflowCount: Counts timer overflows to handle longer durations
 * - lastTicksA, lastTicksB, lastTicksZ: Ticks from the last transition for pulse timing
 * - EncoderRotationDirection: Holds the current direction of rotation (CLOCKWISE or COUNTERCLOCKWISE)
 *
 * This implementation requires proper pin configurations and should be integrated 
 * with a suitable interrupt handling mechanism to ensure accurate readings.
 */

#include "Encoder.h"
volatile uint16_t EncoderPhaseACounter = 0; // Counter for phase A transitions
volatile uint16_t EncoderPhaseBCounter = 0; // Counter for phase B transitions
volatile uint32_t pulseTicksA = 4000000000;          // Duration of pulses for phase A
volatile uint32_t pulseTicksB = 4000000000;          // Duration of pulses for phase B
volatile uint32_t pulseTicksZ = 4294967295;          // Duration of pulses for index signal (Z)

volatile uint16_t TimerOverflowCount = 0;    // To handle overflow for timing accuracy
volatile uint32_t lastTicksA = 0;           // Last recorded ticks for phase A
volatile uint32_t lastTicksB = 0;           // Last recorded ticks for phase B
volatile uint32_t lastTicksZ = 0;           // Last recorded ticks for index signal (Z)
uint8_t TicksATimeCounter;
uint8_t TicksBTimeCounter;

uint8_t EncoderRotationDirection;           // Direction of rotation (CLOCKWISE or COUNTERCLOCKWISE)

void EncoderInit() {
  // Set encoder pins as input with pull-up resistors
  pinMode(ENCODER_PINA, INPUT_PULLUP);
  pinMode(ENCODER_PINB, INPUT_PULLUP);
  pinMode(ENCODER_PINZ, INPUT_PULLUP);
    // Attach pin change interrupts for the encoder phases
  attachPCINT(digitalPinToPCINT(ENCODER_PINA), EncoderPhaseA, FALLING);
  attachPCINT(digitalPinToPCINT(ENCODER_PINB), EncoderPhaseB, FALLING);
  attachPCINT(digitalPinToPCINT(ENCODER_PINZ), EncoderPhaseZ, FALLING);

  // timer configurations
  TCCR1A = 0;                // Configure Timer1 in normal mode (WGM10 and WGM11 bits are 0)
  TCCR1B = 0;                // Set entire TCCR1B register to 0
  TCCR1B |= (1 << CS11);     // Set prescaler to 8 for Timer1 (CS11 = 1, CS10 and CS12 = 0)
  TCNT1 = 0;                 // Start with Timer1 count of 0
  TIMSK1 = (1 << TOIE1);     // Enable Timer1 overflow interrupt
  
  sei(); // enable global interrupts

}

void EncoderPhaseA() {
  uint32_t currentTicksA = (((uint32_t)TimerOverflowCount << 16) + TCNT1 );   // Calculate ticks with overflow handling for 16-bit Timer1
  if((currentTicksA - lastTicksA )> US_TO_TICKS(4)) // if a z pulse detected IN LESS TAN 66 MS (MOTOR SPEED IS ABOVE 1000 RPM ) ignore the result as its a noise
  {
    if(TicksATimeCounter > 99 ){
    TicksATimeCounter = 0 ;
  pulseTicksA = currentTicksA - lastTicksA;                     // Calculate duration of pulse A
  lastTicksA = currentTicksA;                                   // Update last ticks for A
    }else{
      TicksATimeCounter++;
    }

  // Determine the rotation direction based on phase B state
  EncoderRotationDirection = (digitalRead(ENCODER_PINB)) ? COUNTERCLOCKWISE : CLOCKWISE;

  if(EncoderRotationDirection == CLOCKWISE){
    EncoderPhaseACounter++;                     // Increment phase A counter
  } else {
    if(EncoderPhaseACounter == 0){EncoderPhaseACounter = 1500;}
    EncoderPhaseACounter--;                     // Decrement phase A counter
  }
}
}
void EncoderPhaseB() {
  uint32_t currentTicksB = (((uint32_t)TimerOverflowCount << 16) + TCNT1 );   // Calculate ticks with overflow handling
    if((currentTicksB - lastTicksB )> US_TO_TICKS(4)) // if a z pulse detected IN LESS TAN 66 MS (MOTOR SPEED IS ABOVE 1000 RPM ) ignore the result as its a noise
  {
    if(TicksBTimeCounter > 99 ){
    TicksBTimeCounter = 0 ;
  pulseTicksB = currentTicksB - lastTicksB;  // Calculate duration of pulse B
  lastTicksB = currentTicksB;                  // Update last ticks for B
    } else{
      TicksBTimeCounter++;
    }
  // Determine the rotation direction based on phase B state
  EncoderRotationDirection = (digitalRead(ENCODER_PINA)) ? CLOCKWISE : COUNTERCLOCKWISE;

  if(EncoderRotationDirection == CLOCKWISE){
    EncoderPhaseBCounter++;                     // Increment phase B counter
  } else{ 
    if(EncoderPhaseBCounter == 0){EncoderPhaseBCounter = 1500;}
    EncoderPhaseBCounter--;                     // Decrement phase B counter
  }
}
}

void EncoderPhaseZ() {
  
  uint32_t currentTicksZ = (((uint32_t)TimerOverflowCount << 16) + TCNT1 );  // Calculate ticks with overflow handling
  if((currentTicksZ - lastTicksZ )> MS_TO_TICKS(6)) // if a z pulse detected IN LESS TAN 66 MS (MOTOR SPEED IS ABOVE 1000 RPM ) ignore the result as its a noise
  {
  pulseTicksZ = currentTicksZ - lastTicksZ;  // Calculate duration of pulse Z
  lastTicksZ = currentTicksZ;                  // Update last ticks for Z

  //Serial.print("z pulse duration IN US :");
  //Serial.println(TICKS_TO_US(pulseTicksZ));
  // Reset counters and overflow count on Z signal
  EncoderPhaseACounter = 0; 
  EncoderPhaseBCounter = 0; 
}
}

float EncoderGetRPM() {
  // Calculate RPM based on pulseTicksZ duration
  #define NOSA  (ENCODER_PPR/100)
  return (((float)1000000 / TICKS_TO_US(pulseTicksZ)) + ((float)1000000 / TICKS_TO_US(pulseTicksA * NOSA))  + ((float)1000000 / TICKS_TO_US(pulseTicksB * NOSA)) * 60 ) * 20; // Frequency (RPS) * 60 = RPM
}

float EncoderGetAngle() {
  // Calculate the angle based on phase A and B counts

  if ( EncoderPhaseACounter > ENCODER_PPR){ EncoderPhaseACounter-= ENCODER_PPR;}
  if ( EncoderPhaseBCounter > ENCODER_PPR){ EncoderPhaseBCounter-= ENCODER_PPR;}

  
  return (((float)(EncoderPhaseACounter) * (360.0 / ENCODER_PPR)) + 
          ((float)(EncoderPhaseBCounter) * (360.0 / ENCODER_PPR))) / 2.0;

}

bool EncoderGetDirection() {
  // Return the current direction of rotation
  return EncoderRotationDirection;
}


uint16_t EncoderGet_EncoderPhaseACounter(){
  return EncoderPhaseACounter;
}
uint16_t EncoderGet_EncoderPhaseBCounter(){
  return EncoderPhaseBCounter;
}

uint32_t EncoderGet_Clock(){
  return (((uint32_t)TICKS_TO_US(TimerOverflowCount) << 16) + TICKS_TO_US(TCNT1) ) ;
}


// Timer1 overflow interrupt service routine
ISR(TIMER1_OVF_vect) {
  TimerOverflowCount++;  // Increment overflow count on each overflow
}
