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

void setup() {
  
  Serial.begin(115200);
  DisplaySetup();
  StepperSetup(13,12);
  StepperSetSpeed(100);
  StepperDisable  (); 
  delay(100);
  EncoderInit();
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
      //StepperSetAngle(GUIGetAngle ());
      /*
      if(RotationDirection == CLOCKWISE)
      {
        if(EncoderGetAngle()<GUIGetAngle ())
        {
        StepperSetAngle(GUIGetAngle ());
        }
      }
      else {
        if(EncoderGetAngle()>GUIGetAngle ())
        {
        StepperSetAngle(GUIGetAngle ());
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
    if(EncoderGetAngle() != GUIGetAngle ()){
      StepperSetAngle(abs(EncoderGetAngle() - GUIGetAngle ()));
      }
    else{}
  }


  if(CurrentMode == SPEED_CONTROL && ( (millis() - UpdateMillis ) > CORRECTION_PERIOD_FOR_SPEED_CONTROL)  ){
    UpdateMillis = millis();
    //StepperSetSpeed(GUIGetRPM () + ( GUIGetRPM () - EncoderGetRPM())); 
    StepperSetSpeed(GUIGetRPM () ); 
     Serial.print("encoder Read Speed:");
    Serial.println(EncoderGetRPM());
  }

}

