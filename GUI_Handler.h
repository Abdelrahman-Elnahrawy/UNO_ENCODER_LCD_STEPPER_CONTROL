
#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Arduino.h>
#include <LiquidCrystal.h>

// External LCD object
extern LiquidCrystal lcd;

// Function prototypes
void DisplaySetup();
void DisplaySelectionScreen(char buttonState);
void displaySpeed(float speed, bool Direction);
void displayAngle(float angle, bool Direction);
void DisplayDirection(bool Direction);
void DisplayUpdate(char command);
unsigned int readLCDButtons();
bool GUIGetDirection();

void DisplaySpeedScreen(char command);
void DisplayAngleScreen(char command);
void DisplaySelectionScreen(char command);
void DisplayCursorPositionUpdate(char command);
void DisplayChangeSpeed(char command);
void DisplayChangeAngle(char command);


float GUIGetRPM ();
float GUIGetAngle ();
char GUIGetMode();

void DisplayLockUpdate ();
// Button constants
#define BUTTON_NULL    0
#define BUTTON_RIGHT   1
#define BUTTON_LEFT    2
#define BUTTON_UP      3
#define BUTTON_DOWN    4
#define BUTTON_SELECT  5
#define BUTTON_SELECT_HOLD  6

#define UPDATE_ALL_SCREEN 6 // used when another mode is set for the first time 
// Direction constants
#define CLOCKWISE        true
#define COUNTERCLOCKWISE false

#define MODE_SELECT    2
#define SPEED_CONTROL  1
#define ANGLE_CONTROL  0

#define SCREEN_SELECT 2
#define SCREEN_SPEED  1
#define SCREEN_ANGLE  0

#define MAX_SPEED 3876

#endif
