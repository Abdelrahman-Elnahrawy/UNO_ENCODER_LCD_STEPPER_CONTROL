#include "GUI_Handler.h"
#include "Stepper_Handler.h"
#include "Encoder.h"
#include "FreqGenPin11.h"
// Control modes


#define SCREEN_SELECTION 0
#define SCREEN_SPEED     1
#define SCREEN_ANGLE     2
uint8_t CurrentMode;
uint32_t UpdateMillis;

#define CORRECTION_PERIOD_FOR_SPEED_CONTROL   500 //in ms
#define CORRECTION_PERIOD_FOR_ANGLE_CONTROL   500 //in ms


// PID Control Variables
static float rpmIntegral = 0;        // Accumulated RPM error (integral term)
static float previousRPMError = 0;   // Previous RPM error for derivative calculation

// PID Gains (Tuning Parameters)
float rpmKp = 0.1; // Proportional gain for RPM control
float rpmKi = 0.05; // Integral gain for RPM control
float rpmKd = 0.01; // Derivative gain for RPM control


void setup() {
  
  Serial.begin(115200);
  DisplaySetup();
  StepperSetup(13,12);
  StepperSetSpeed(100);
  StepperDisable  (); 
  delay(100);
  EncoderInit();
  delay(100);
}

void loop() {
delay(500);
Serial.print("Current Angle = ");
Serial.println(EncoderGetAngle());

Serial.print("EncoderPhaseACounter = ");
Serial.println(EncoderGet_EncoderPhaseACounter());

Serial.print("EncoderPhaseBCounter = ");
Serial.println(EncoderGet_EncoderPhaseBCounter());

Serial.print("time in ms ");
Serial.println((EncoderGet_Clock()/1000));

Serial.print("rotation direction is ");
if(EncoderGetDirection() == CLOCKWISE){
  Serial.println("CLOCKWISE");
}
else{
  Serial.println("Counter CLOCKWISE");
}
EncoderGetDirection();
}
