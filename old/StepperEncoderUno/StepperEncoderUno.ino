#include "GUI_Handler.h"
#include "Stepper_Handler.h"
#include "Encoder.h"
#include "FreqGenPin11.h"



#define GEAR_RATIO       1
#define SET_ANGLE_SPEED  200



uint8_t CurrentMode;
uint32_t UpdateMillis;
float MoveAngle;
float CurrentSpeedFactor = 1;


// PID Control Variables
static float rpmIntegral = 0;        // Accumulated RPM error (integral term)
static float previousRPMError = 0;   // Previous RPM error for derivative calculation

// PID Gains (Tuning Parameters)
float rpmKp = 0.2;  // Proportional gain for RPM control
float rpmKi = 0.05; // Integral gain for RPM control
float rpmKd = 0.32; // Derivative gain for RPM control


#define CORRECTION_PERIOD_FOR_SPEED_CONTROL   500 //in ms
#define CORRECTION_PERIOD_FOR_ANGLE_CONTROL   10 //in ms

void setup() {
  
  Serial.begin(115200);
  DisplaySetup();
  EncoderInit();
  StepperSetup(13,12);
  StepperDisable  (); 



  StepperEnable  ();
  StepperSetAngle(360, 60);
  delay(5000);

  while( EncoderGetAngle() > 2 ){
  Serial.println(EncoderGetAngle());
  StepperSetAngle(1.8, SET_ANGLE_SPEED);
  delay(50);
  }
  StepperDisable  (); 

}

void loop() {
  char buttonState = readLCDButtons(); // Read button state
  if (buttonState != BUTTON_NULL) {
    DisplayUpdate( buttonState );
    CurrentMode = GUIGetMode();
    bool RotationDirection = GUIGetDirection();
    StepperSetDirection(RotationDirection);

    switch (CurrentMode){
      case MODE_SELECT:
      StepperDisable  ();    
      break;
      case SPEED_CONTROL:
      StepperEnable  ();
      Serial.println("speed Control Mode");
      //StepperSetSpeed(GUIGetRPM ());
      break;
      case ANGLE_CONTROL:
      StepperEnable  ();
      Serial.println("Angle Control Mode");
      if(buttonState == BUTTON_SELECT){
      MoveAngle = fmod((EncoderGetAngle() + GUIGetAngle ()) , 360);}
      //StepperSetAngle(GUIGetAngle (), SET_ANGLE_SPEED);
      /*
      if(RotationDirection == CLOCKWISE)
      {
        if(EncoderGetAngle()<GUIGetAngle ())
        {
        StepperSetAngle(GUIGetAngle (), SET_ANGLE_SPEED);
        }
      }
      else {
        if(EncoderGetAngle()>GUIGetAngle ())
        {
        StepperSetAngle(GUIGetAngle (), SET_ANGLE_SPEED);
        }
      }
      */
      break;
      default:
      break;
    }
  }
  


  if(CurrentMode == ANGLE_CONTROL && ( (millis() - UpdateMillis ) > CORRECTION_PERIOD_FOR_ANGLE_CONTROL)  ){
    UpdateMillis = millis();
    if(abs(MoveAngle  - EncoderGetAngle())>STEP_ANGLE * 2 ){

      Serial.print("current angle is :");
      Serial.println(EncoderGetAngle());

      //Serial.print("EncoderPhaseACounter :");
      //Serial.print(EncoderGet_EncoderPhaseACounter());
      //Serial.print(" EncoderPhaseBCounter");
      //Serial.print(EncoderGet_EncoderPhaseBCounter());

      Serial.print("  MoveAngle is:");
      Serial.print(MoveAngle);
      Serial.print("  Stepper Set Angle is:");
      Serial.println(abs((MoveAngle  - EncoderGetAngle())/2));
      StepperSetAngle(abs((MoveAngle  - EncoderGetAngle())/2), SET_ANGLE_SPEED);
  }
  else if (abs(MoveAngle  - EncoderGetAngle())>STEP_ANGLE){
    StepperSetAngle( abs( ( MoveAngle  - EncoderGetAngle() ) ) , SET_ANGLE_SPEED);

  }



  }



  if(CurrentMode == SPEED_CONTROL && ( (millis() - UpdateMillis ) > CORRECTION_PERIOD_FOR_SPEED_CONTROL)  ){
    UpdateMillis = millis();
   Serial.print("Actual speed:");
   Serial.print(EncoderGetRPM());
   //Serial.print(" factor:");
   //Serial.print(CurrentSpeedFactor);
   Serial.print("  Set speed:");
   Serial.println(GUIGetRPM () );
   Serial.println();

    // Get Target and Actual RPM
    float rpmError = GUIGetRPM() - EncoderGetRPM(); // Calculate RPM error
    // PID Output Calculation
    rpmIntegral += rpmError; // Update integral term with current error
    float rpmDerivative = rpmError - previousRPMError; // Calculate change in error (derivative term)
    float rpmOutput = (rpmKp * rpmError) + (rpmKi * rpmIntegral) + (rpmKd * rpmDerivative);

// Set the stepper motor speed
//StepperSetSpeed(EncoderGetRPM() + rpmOutput);
StepperSetSpeed(GUIGetRPM());
// Update previous error for next iteration
previousRPMError = rpmError;
  }
}
