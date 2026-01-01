#ifndef STEPPER_HANDLER_H
#define STEPPER_HANDLER_H

#include <Arduino.h>

#define STEPS_PER_REVOLUTION    200 
#define STEP_ANGLE       1.8
void StepperSetup(char EnablePin , char DirectionPin );
void StepperEnable  ();
void StepperDisable();
void StepperSetSpeed(float speed);
void StepperSetAngle(float angle ,uint16_t speed);
void StepperSetDirection(bool direction);
#endif