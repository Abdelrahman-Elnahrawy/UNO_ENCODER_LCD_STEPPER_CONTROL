
#include "Stepper_Handler.h"
#include "FreqGenPin11.h"

static char StepperEnablePin ;
static char StepperDirectionPin ;
uint32_t StepsPerSecond;



void StepperSetup(char EnablePin , char DirectionPin ){
StepperEnablePin    = EnablePin   ;
StepperDirectionPin = DirectionPin;
pinMode(StepperEnablePin   , OUTPUT);
pinMode(StepperDirectionPin, OUTPUT);
FreqGenPin11Init();
}

void StepperEnable  (){
  digitalWrite(StepperEnablePin,LOW);
}
void StepperDisable(){
  digitalWrite(StepperEnablePin,HIGH);
  DisableFreqGenPin11();
}
void StepperSetDirection(bool direction){
  digitalWrite(StepperDirectionPin,direction);
}

void StepperSetSpeed(float speed){
  if(speed == 0){
   DisableFreqGenPin11();
  }
 // Serial.print("\n the Current Speed given to stepper is:");
  //Serial.println(speed);
  StepsPerSecond =((speed *2* STEPS_PER_REVOLUTION)/60);
 // Serial.print("\n the number if steps per second is:");
  //Serial.println(StepsPerSecond);
  setFreqPin11(StepsPerSecond);  //the speed is in rpm .. to get the revolution per second = speed / 60 ,,, and to get the amount of steps required
}

void StepperSetAngle(float angle , uint16_t speed){
  FreqGenGeneratePulses((int)((angle * 300) /360), speed);
}