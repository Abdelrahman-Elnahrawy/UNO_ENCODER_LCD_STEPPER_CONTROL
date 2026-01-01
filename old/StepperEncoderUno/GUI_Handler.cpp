
#include "GUI_Handler.h"
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

char CurrentScreen   = MODE_SELECT;
bool selectedControl = 1;
float Angle = 0 ;
float Speed = 0 ;
bool RotationDirection = 0 ;
int  CursorPosition = 7 ;
bool LockScreen = 1;
void DisplaySetup(){
    lcd.begin(16, 2);
    lcd.clear();
    DisplayUpdate(UPDATE_ALL_SCREEN);
    Serial.println("Setup completed\n");
}


void DisplayDirection (bool Direction){
    lcd.setCursor(14, 1);
    if(Direction == CLOCKWISE){
     lcd.print("CW");
    } else {
      lcd.print("CC");                  
    }
    lcd.setCursor(CursorPosition, 1);  
}


unsigned int readLCDButtons() {
  
    int button = BUTTON_NULL;
    // Check if any button is pressed (analog value less than 1000)
    if (analogRead(0) < 1020) {
        delay(100);  // Short delay for debouncing

        int adcKeyIn = analogRead(0);  // Read the analog value again

        // Check for button presses based on analog values
        if (adcKeyIn < 50) {
            button = BUTTON_RIGHT;
        } else if (adcKeyIn < 200) {
            button = BUTTON_UP;
        } else if (adcKeyIn < 380) {
            button = BUTTON_DOWN;
        } else if (adcKeyIn < 600) {
            button = BUTTON_LEFT;
        } else if (adcKeyIn < 800) {
            button = BUTTON_SELECT;
        }
          int counter = 0;
        while (analogRead(0) < 1000) {
            counter ++;
            delay(100);
            if(button == BUTTON_SELECT && counter > 20){
              Serial.println("Switch to selection screen");
              CurrentScreen = SCREEN_SELECT;
              button = BUTTON_SELECT_HOLD;
              lcd.noCursor();
              LockScreen =1;
              DisplayUpdate(BUTTON_SELECT_HOLD);
            }
            if (counter>100){
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("   Stuck button ");
              lcd.setCursor(CursorPosition, 1);  
            } 
        }
        // Wait until the button is released

    }

    // Debug print for button state
    switch (button) {
        case BUTTON_RIGHT: 
            Serial.println("Button RIGHT"); 
            break;
        case BUTTON_LEFT: 
            Serial.println("Button LEFT"); 
            break;
        case BUTTON_UP: 
            Serial.println("Button UP"); 
            break;
        case BUTTON_DOWN: 
            Serial.println("Button DOWN"); 
            break;
        case BUTTON_SELECT: 
            Serial.println("Button SELECT"); 
            break;
        default: 
            break;
    }
    return button;
}



void DisplayUpdate(char command)
{ Serial.println("Display Update");
  if(command == BUTTON_SELECT){
    Serial.println("select Button is used !");
    switch (CurrentScreen){
      case SPEED_CONTROL:
      DisplayLockUpdate();
      break;
      case ANGLE_CONTROL:
      DisplayLockUpdate();
      break;
      default :
      Serial.println("select Button is used !& current screen is mode selection");
      if(selectedControl == SPEED_CONTROL){
        Serial.println("Switch to speed screen");
        CurrentScreen = SCREEN_SPEED;
        DisplaySpeedScreen(UPDATE_ALL_SCREEN);  
      } else{
        CurrentScreen = SCREEN_ANGLE;
        Serial.println("Switch to Angle screen");      
        DisplayAngleScreen(UPDATE_ALL_SCREEN);  
      }
      DisplayLockUpdate ();
      break;
    }  
  }else{
    switch (CurrentScreen){
      case SCREEN_SPEED:
      DisplaySpeedScreen(command);
      break;
      case SCREEN_ANGLE:
      DisplayAngleScreen(command);
      break;
      default :
      Serial.println("Switch to selection screen");
      DisplaySelectionScreen(command);
      break;
    }
  }

  if(CurrentScreen != SCREEN_SELECT){
    DisplayDirection (RotationDirection);
  }

}



void DisplaySpeedScreen(char command){
  Serial.println("in speed screen");  
  if(command == UPDATE_ALL_SCREEN){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("   Speed Mode");
    lcd.setCursor(0, 1);
    lcd.print("Speed: ");
  }
  // Format speed display
  DisplayCursorPositionUpdate(command);
  DisplayChangeSpeed(command);
  int speedInteger = Speed;
  lcd.setCursor(6, 1);
  lcd.print(speedInteger > 999 ? speedInteger / 1000 : 0);
  lcd.print((speedInteger % 1000) / 100);
  lcd.print((speedInteger % 100) / 10);
  lcd.print(speedInteger % 10);
  lcd.print(".");
  lcd.print((int)((Speed - floorf(Speed)) * 10));
  lcd.setCursor(CursorPosition, 1);  
}

void DisplayAngleScreen(char command){
  Serial.println("in Angle screen");  
    if(command == UPDATE_ALL_SCREEN){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("   Angle Mode");
      lcd.setCursor(0, 1);
      lcd.print("Angle:  ");
    }
    // Format speed display
    DisplayCursorPositionUpdate(command);
    DisplayChangeAngle(command);
    int angle_Integer = Angle;
    lcd.setCursor(7, 1);
    lcd.print((angle_Integer % 1000) / 100);
    lcd.print((angle_Integer % 100) / 10);
    lcd.print(angle_Integer % 10);
    lcd.print(".");
    lcd.print((int)((Angle - floorf(Angle)) * 10));
    lcd.setCursor(CursorPosition, 1);  
}

void DisplaySelectionScreen(char command){
  if(command == UPDATE_ALL_SCREEN){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" CONTROL TYPE");
  }

  if (command == BUTTON_RIGHT || command == BUTTON_LEFT) {
    selectedControl = (selectedControl == SPEED_CONTROL) ? ANGLE_CONTROL : SPEED_CONTROL;
  } 
  lcd.setCursor(0, 1);
  Serial.print("in selection screen,selectedControl =");
  Serial.println((int)selectedControl);
  lcd.print((selectedControl == SPEED_CONTROL ) ? "> SPEED    ANGLE " : "  SPEED  > ANGLE ");
  lcd.setCursor(CursorPosition, 1);  
}

void DisplayCursorPositionUpdate(char command){
  int MinCursorPosition = (CurrentScreen == SCREEN_SPEED)? 6:7 ;
  switch (command){
    case  BUTTON_LEFT:
    CursorPosition--;
    if(CursorPosition == 10 || CursorPosition == 12 ){CursorPosition --;}
    if (CursorPosition < MinCursorPosition) {CursorPosition = 13;}
    break;
    case BUTTON_RIGHT:
    CursorPosition++;
    if(CursorPosition == 10 || CursorPosition == 12 ){CursorPosition ++;}
    if (CursorPosition > 13) {CursorPosition = MinCursorPosition;}
    break;
    default:
    break;
  }
  Serial.print("cursor now at:");
  Serial.println(CursorPosition);
  lcd.setCursor(13, 1);
  if (CursorPosition == 13) {
    lcd.print(">");
    lcd.noCursor();
    }
    else {
    lcd.print(" ");
    lcd.cursor();  
      }
  lcd.setCursor(CursorPosition, 1); 
}



void DisplayChangeSpeed(char command){
  switch (command){
    case BUTTON_UP:
      switch (CursorPosition){
        case 6:
        Speed+=1000;
        break;
        case 7:
        Speed+=100;
        break;
        case 8:
        Speed+= 10;        
        break;
        case 9:
        Speed+=  1;        
        break;
        case 11:
        Speed+=0.1;        
        break;
        case 13:
        RotationDirection = !(RotationDirection);
        DisplayDirection (RotationDirection);   
        break;
      }
    break;
    case BUTTON_DOWN:
      switch (CursorPosition){
        case 6:
        Speed-=1000;
        break;
        case 7:
        Speed-=100;
        break;
        case 8:
        Speed-= 10;        
        break;
        case 9:
        Speed-=  1;        
        break;
        case 11:
        Speed-=0.1;        
        break;
        case 13:
        RotationDirection = !(RotationDirection);
        DisplayDirection (RotationDirection);   
        break;
      }
    break;
    default:
    break;
  }
  if (Speed> MAX_SPEED){Speed = MAX_SPEED;}
  if (Speed< 0        ){Speed = 0;        }
  
}

void DisplayChangeAngle(char command){
  switch (command){
    case BUTTON_UP:
      switch (CursorPosition){
        case 7:
        Angle+=100;
        break;
        case 8:
        Angle+= 10;        
        break;
        case 9:
        Angle+=  1;        
        break;
        case 11:
        Angle+=0.1;        
        break;
        case 13:
        RotationDirection = !(RotationDirection);
        DisplayDirection (RotationDirection);   
        break;
      }
    break;
    case BUTTON_DOWN:
      switch (CursorPosition){
        case 7:
        Angle-=100;
        break;
        case 8:
        Angle-= 10;        
        break;
        case 9:
        Angle-=  1;        
        break;
        case 11:
        Angle-=0.1;        
        break;
        case 13:
        RotationDirection = !(RotationDirection);
        DisplayDirection (RotationDirection);   
        break;
      }
    break;
    default:
    break;
  }
  if(Angle>360){Angle -=360 ;}
  if(Angle<  0){Angle +=360 ;}
}

void DisplayLockUpdate (){
        if(LockScreen){ LockScreen =0; lcd.cursor();    }
        else          { LockScreen =1; lcd.noCursor();  }
}
float GUIGetRPM (){
  return Speed;
}
float GUIGetAngle (){
  if (Angle == 0){
    return 360;
  }
  else{
  return Angle;
  }
}
char GUIGetMode(){
  return CurrentScreen;
}
bool GUIGetDirection(){
  return RotationDirection;
}
