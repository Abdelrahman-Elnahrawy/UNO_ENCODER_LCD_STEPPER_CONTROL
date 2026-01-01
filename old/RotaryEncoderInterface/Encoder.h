#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include "PinChangeInterrupt.h"

#define ENCODER_PINA  A1    // A1 
#define ENCODER_PINB  A2    // A2
#define ENCODER_PINZ  A3    // A3
#define ENCODER_PPR   1500  // Pulses per revolution for the encoder

#define ENCODER_TIMER_FREQ 2000000
#ifndef COUNTERCLOCKWISE 
#define COUNTERCLOCKWISE 1
#endif

#ifndef CLOCKWISE 
#define CLOCKWISE 0
#endif

#define TICKS_TO_MS(ticks) ((ticks) / 2000.0)
#define TICKS_TO_US(ticks) ((ticks) / 2)


void EncoderInit();

void EncoderPhaseA();
void EncoderPhaseB();
void EncoderPhaseZ();

float EncoderGetRPM();
float EncoderGetAngle();

bool EncoderGetDirection();

uint16_t EncoderGet_EncoderPhaseACounter();
uint16_t EncoderGet_EncoderPhaseBCounter();
uint32_t EncoderGet_Clock();
#endif