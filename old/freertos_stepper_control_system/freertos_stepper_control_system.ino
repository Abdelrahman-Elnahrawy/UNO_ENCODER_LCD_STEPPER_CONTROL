/*
 * Project: RTOS-Based Stepper Motor Controller
 * Author: Abdelrahman Elnahrawy
 * Hardware: Arduino + LCD Keypad Shield + Stepper Driver (A4988/DRV8825)
 * OS: FreeRTOS
 */

#include <LiquidCrystal.h>
#include <Arduino_FreeRTOS.h> // Ensure you have this library installed

// --- Pin Definitions ---
#define stepPin 3
#define dirPin 4
#define enablePin 5

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Global State (Thread-Safe considerations needed for production)
volatile int mode = 0; 
volatile float motorParam = 0; // Shared variable for Speed or Angle

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); 

  lcd.begin(16, 2);

  // Task Creation: Name, Stack Size, Priority
  xTaskCreate(vLCDTask,     "LCD",     128, NULL, 2, NULL);
  xTaskCreate(vStepperTask, "Stepper", 128, NULL, 3, NULL); // Higher priority
  xTaskCreate(vEncoderTask, "Encoder", 128, NULL, 1, NULL);

  // Scheduler starts automatically in Arduino FreeRTOS
}

void loop() {} // Empty as per RTOS standards

// --- Task Implementations ---

void vStepperTask(void *pvParameters) {
  for (;;) {
    if (mode == 1) { // Speed Mode logic
      digitalWrite(stepPin, HIGH);
      vTaskDelay(1); // Real-time pulse control
      digitalWrite(stepPin, LOW);
      vTaskDelay(1);
    }
    // Task yields to others
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void vLCDTask(void *pvParameters) {
  for (;;) {
    // Handling button logic and menu updates
    vTaskDelay(pdMS_TO_TICKS(200)); 
  }
}

void vEncoderTask(void *pvParameters) {
  for (;;) {
    // Logic for rotary encoder increment/decrement
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}