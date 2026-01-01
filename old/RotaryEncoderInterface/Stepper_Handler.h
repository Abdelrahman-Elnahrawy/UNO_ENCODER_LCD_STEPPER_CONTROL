#ifndef STEPPER_HANDLER_H
#define STEPPER_HANDLER_H

#include <Arduino.h>

#define STEPS_PER_REVOLUTION    200 
void StepperSetup(char EnablePin , char DirectionPin );
void StepperEnable  ();
void StepperDisable();
void StepperSetSpeed(float speed);
void StepperSetAngle(float angle);
void StepperSetDirection(bool direction);
#endif