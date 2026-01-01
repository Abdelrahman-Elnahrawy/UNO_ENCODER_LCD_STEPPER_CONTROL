/*
  Project: LCD Keypad Stepper Controller
  Board:   Arduino Uno / Nano (ATmega328P)
  Author:  Abdelrahman Elnahrawy
  License: MIT

  Description:
  - LCD Keypad Shield UI (A0 buttons) for controlling a stepper in speed or angle mode.
  - Rotary encoder with A/B phases and optional Z index provides feedback (PinChangeInterrupt).
  - Step pulses generated on Pin 11 using Timer2 CTC toggle on OC2A only.

  Improvements in this version:
  - Parameterized encoder wraps using ENCODER_PPR (no hardcoded 1500).
  - Timer2 generator toggles only OC2A; OC2B (Pin 3) is not used.
  - Clear constants for pins, timings, and limits; structured code and comments.

  Notes:
  - LCD Keypad Shield uses pins 8,9,4,5,6,7 and A0 for buttons.
  - Encoder A/B/Z on A1/A2/A3 with INPUT_PULLUP.
  - Stepper driver: Enable (D13), Direction (D12), Step on D11 via Timer2.
*/

#include <Arduino.h>
#include <LiquidCrystal.h>
#include "PinChangeInterrupt.h"

// -----------------------------
// LCD Keypad shield
// -----------------------------
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
static const uint8_t ADC_BUTTON_PIN = A0; // analog ladder

// Button codes
enum Button : uint8_t {
  BUTTON_NULL = 0, BUTTON_RIGHT, BUTTON_LEFT, BUTTON_UP, BUTTON_DOWN, BUTTON_SELECT, BUTTON_SELECT_HOLD
};

// Screen modes
enum ScreenMode : uint8_t { SCREEN_ANGLE = 0, SCREEN_SPEED = 1, SCREEN_SELECT = 2 };
static ScreenMode current_screen = SCREEN_SELECT;
static bool rotation_cw = true; // CW=true, CCW=false
static bool lock_cursor = true; // select toggles cursor lock
static int cursor_pos = 7;      // editable field position
static bool selected_speed_mode = true;

// -----------------------------
// Encoder (A/B/Z) with pin change interrupts
// -----------------------------
static const uint8_t ENCODER_PINA = A1; // Phase A
static const uint8_t ENCODER_PINB = A2; // Phase B
static const uint8_t ENCODER_PINZ = A3; // Index Z (optional)
static const uint16_t ENCODER_PPR = 1500; // pulses per revolution for your encoder

// Timer1 (16-bit) prescaler = 8, used as a free-running timebase
// Macros for converting ticks to time (16 MHz / 8 = 2 MHz => 0.5 us per tick)
#define TICKS_TO_MS(ticks) ((ticks) / 2000.0)
#define TICKS_TO_US(ticks) ((ticks) / 2)

volatile uint16_t enc_count_a = 0;
volatile uint16_t enc_count_b = 0;
volatile uint32_t last_ticks_a = 0, last_ticks_b = 0, last_ticks_z = 0;
volatile uint32_t pulse_ticks_z = 0;
volatile uint16_t ovf_count = 0;
volatile bool enc_dir_cw = true; // true=CW, false=CCW

ISR(TIMER1_OVF_vect) { ovf_count++; }

static inline uint32_t ticks_now() {
  return (uint32_t(ovf_count) << 16) + TCNT1; // extend 16-bit TCNT1 with overflow count
}

void isr_phase_a() {
  uint32_t now = ticks_now();
  // Direction: read other phase
  enc_dir_cw = (digitalRead(ENCODER_PINB) == LOW); // falling on A, if B low => CW
  if (enc_dir_cw) {
    enc_count_a = (enc_count_a + 1) % ENCODER_PPR;
  } else {
    enc_count_a = (enc_count_a == 0) ? (ENCODER_PPR - 1) : (enc_count_a - 1);
  }
  last_ticks_a = now;
}

void isr_phase_b() {
  uint32_t now = ticks_now();
  enc_dir_cw = (digitalRead(ENCODER_PINA) == HIGH); // falling on B, if A high => CW
  if (enc_dir_cw) {
    enc_count_b = (enc_count_b + 1) % ENCODER_PPR;
  } else {
    enc_count_b = (enc_count_b == 0) ? (ENCODER_PPR - 1) : (enc_count_b - 1);
  }
  last_ticks_b = now;
}

void isr_phase_z() {
  uint32_t now = ticks_now();
  pulse_ticks_z = now - last_ticks_z;
  last_ticks_z = now;
  enc_count_a = 0;
  enc_count_b = 0;
  ovf_count = 0;
  TCNT1 = 0;
}

float encoder_get_angle_deg() {
  // Average A/B into degrees
  float a_deg = (float)enc_count_a * (360.0f / ENCODER_PPR);
  float b_deg = (float)enc_count_b * (360.0f / ENCODER_PPR);
  return (a_deg + b_deg) * 0.5f;
}

float encoder_get_rpm_from_z() {
  if (pulse_ticks_z == 0) return 0.0f;
  const float timer_hz = 2000000.0f; // 2 MHz with prescaler 8
  float rps = (timer_hz / (float)pulse_ticks_z);
  return rps * 60.0f;
}

// -----------------------------
// Timer2-based step pulse generator on OC2A (Pin 11)
// -----------------------------
static const uint8_t STEPPER_STEP_PIN = 11;    // OC2A toggles
static const uint8_t STEPPER_DIR_PIN  = 12;    // direction
static const uint8_t STEPPER_EN_PIN   = 13;    // enable, LOW=on for common drivers

static uint8_t t2_prescaler_bits = 0;

void stepgen_init() {
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  // Timer2 CTC, toggle OC2A, stop timer initially
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1 << WGM21);  // CTC mode
  TCCR2A |= (1 << COM2A0); // toggle OC2A on match
  // Do NOT set COM2B0 (Pin 3). We only use OC2A (Pin 11).
}

void stepgen_disable() {
  // Stop Timer2 by clearing prescaler bits
  TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
}

void stepgen_enable() {
  TCCR2B = t2_prescaler_bits; // resume last config
}

void stepgen_set_hz(float frequency_hz) {
  if (frequency_hz <= 0) { stepgen_disable(); return; }
  unsigned long top;
  if (frequency_hz >= 31373) {
    t2_prescaler_bits = (1 << CS20);
    top = (16000000UL / (1UL * 2UL * (unsigned long)frequency_hz)) - 1UL;
  } else if (frequency_hz >= 3921) {
    t2_prescaler_bits = (1 << CS21);
    top = (16000000UL / (8UL * 2UL * (unsigned long)frequency_hz)) - 1UL;
  } else if (frequency_hz >= 980) {
    t2_prescaler_bits = (1 << CS21) | (1 << CS20);
    top = (16000000UL / (32UL * 2UL * (unsigned long)frequency_hz)) - 1UL;
  } else if (frequency_hz >= 490) {
    t2_prescaler_bits = (1 << CS22);
    top = (16000000UL / (64UL * 2UL * (unsigned long)frequency_hz)) - 1UL;
  } else if (frequency_hz >= 245) {
    t2_prescaler_bits = (1 << CS22) | (1 << CS20);
    top = (16000000UL / (128UL * 2UL * (unsigned long)frequency_hz)) - 1UL;
  } else if (frequency_hz >= 122) {
    t2_prescaler_bits = (1 << CS22) | (1 << CS21);
    top = (16000000UL / (256UL * 2UL * (unsigned long)frequency_hz)) - 1UL;
  } else {
    t2_prescaler_bits = (1 << CS22) | (1 << CS21) | (1 << CS20);
    top = (16000000UL / (1024UL * 2UL * (unsigned long)frequency_hz)) - 1UL;
  }
  if (top > 255UL) top = 255UL; // 8-bit timer
  OCR2A = (uint8_t)top;
  TCCR2B = t2_prescaler_bits;
  TCCR2A = (1 << WGM21) | (1 << COM2A0); // CTC + toggle OC2A
}

void stepper_setup() {
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, HIGH); // disabled by default (LOW=enable)
  stepgen_init();
}

void stepper_enable(bool en) {
  digitalWrite(STEPPER_EN_PIN, en ? LOW : HIGH);
  if (!en) stepgen_disable();
}

void stepper_set_direction(bool cw) {
  digitalWrite(STEPPER_DIR_PIN, cw ? HIGH : LOW);
}

static const uint16_t STEPS_PER_REV = 200; // set to your motor (e.g., 200 steps = 1.8 deg)

void stepper_set_speed_rpm(float rpm) {
  if (rpm <= 0) { stepgen_disable(); return; }
  float steps_per_s = (rpm * STEPS_PER_REV) / 60.0f; // convert to frequency
  stepgen_set_hz(steps_per_s);
}

// -----------------------------
// LCD UI helpers
// -----------------------------
static const float MAX_SPEED_RPM = 3876.0f; // inherited project limit
static float ui_angle_deg = 0.0f;
static float ui_speed_rpm = 0.0f;

void lcd_setup_screen() {
  lcd.begin(16, 2);
  lcd.clear();
  // initial screen
  lcd.setCursor(0, 0); lcd.print(" CONTROL TYPE");
  lcd.setCursor(0, 1); lcd.print("> SPEED    ANGLE ");
}

void lcd_show_direction(bool cw) {
  lcd.setCursor(14, 1);
  lcd.print(cw ? "CW" : "CC");
  lcd.setCursor(cursor_pos, 1);
}

Button read_lcd_buttons() {
  if (analogRead(ADC_BUTTON_PIN) >= 1020) return BUTTON_NULL;
  delay(100); // debouncing
  int adc = analogRead(ADC_BUTTON_PIN);
  if      (adc <  50) return BUTTON_RIGHT;
  else if (adc < 200) return BUTTON_UP;
  else if (adc < 380) return BUTTON_DOWN;
  else if (adc < 600) return BUTTON_LEFT;
  else if (adc < 800) return BUTTON_SELECT;
  // hold select handling
  int counter = 0;
  while (analogRead(ADC_BUTTON_PIN) < 1000) {
    counter++;
    delay(100);
    if (counter > 20) {
      current_screen = SCREEN_SELECT;
      lcd.noCursor();
      lock_cursor = true;
      return BUTTON_SELECT_HOLD;
    }
    if (counter > 100) {
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("   Stuck button ");
    }
  }
  return BUTTON_NULL;
}

void display_selection(Button cmd) {
  if (cmd == BUTTON_LEFT || cmd == BUTTON_RIGHT) {
    selected_speed_mode = !selected_speed_mode;
  }
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(" CONTROL TYPE");
  lcd.setCursor(0, 1);
  lcd.print(selected_speed_mode ? "> SPEED    ANGLE " : "  SPEED  > ANGLE ");
}

void display_speed(Button cmd) {
  // Cursor movement
  int min_cursor = 6;
  if (cmd == BUTTON_LEFT)  { cursor_pos--; if (cursor_pos == 10 || cursor_pos == 12) cursor_pos--; if (cursor_pos < min_cursor) cursor_pos = 13; }
  if (cmd == BUTTON_RIGHT) { cursor_pos++; if (cursor_pos == 10 || cursor_pos == 12) cursor_pos++; if (cursor_pos > 13) cursor_pos = min_cursor; }
  // Value changes
  if (cmd == BUTTON_UP) {
    if      (cursor_pos == 6)  ui_speed_rpm += 1000.0f;
    else if (cursor_pos == 7)  ui_speed_rpm += 100.0f;
    else if (cursor_pos == 8)  ui_speed_rpm += 10.0f;
    else if (cursor_pos == 9)  ui_speed_rpm += 1.0f;
    else if (cursor_pos == 11) ui_speed_rpm += 0.1f;
    else if (cursor_pos == 13) rotation_cw = !rotation_cw;
  }
  if (cmd == BUTTON_DOWN) {
    if      (cursor_pos == 6)  ui_speed_rpm -= 1000.0f;
    else if (cursor_pos == 7)  ui_speed_rpm -= 100.0f;
    else if (cursor_pos == 8)  ui_speed_rpm -= 10.0f;
    else if (cursor_pos == 9)  ui_speed_rpm -= 1.0f;
    else if (cursor_pos == 11) ui_speed_rpm -= 0.1f;
    else if (cursor_pos == 13) rotation_cw = !rotation_cw;
  }
  if (ui_speed_rpm < 0) ui_speed_rpm = 0;
  if (ui_speed_rpm > MAX_SPEED_RPM) ui_speed_rpm = MAX_SPEED_RPM;

  // Draw
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("   Speed Mode");
  lcd.setCursor(0, 1); lcd.print("Speed: ");
  int si = (int)ui_speed_rpm;
  lcd.print(si > 999 ? si / 1000 : 0);
  lcd.print((si % 1000) / 100);
  lcd.print((si % 100) / 10);
  lcd.print(si % 10);
  lcd.print(".");
  lcd.print((int)((ui_speed_rpm - floorf(ui_speed_rpm)) * 10));
  lcd.setCursor(cursor_pos, 1);
}

void display_angle(Button cmd) {
  int min_cursor = 7;
  if (cmd == BUTTON_LEFT)  { cursor_pos--; if (cursor_pos == 10 || cursor_pos == 12) cursor_pos--; if (cursor_pos < min_cursor) cursor_pos = 13; }
  if (cmd == BUTTON_RIGHT) { cursor_pos++; if (cursor_pos == 10 || cursor_pos == 12) cursor_pos++; if (cursor_pos > 13) cursor_pos = min_cursor; }
  if (cmd == BUTTON_UP) {
    if      (cursor_pos == 7)  ui_angle_deg += 100.0f;
    else if (cursor_pos == 8)  ui_angle_deg += 10.0f;
    else if (cursor_pos == 9)  ui_angle_deg += 1.0f;
    else if (cursor_pos == 11) ui_angle_deg += 0.1f;
    else if (cursor_pos == 13) rotation_cw = !rotation_cw;
  }
  if (cmd == BUTTON_DOWN) {
    if      (cursor_pos == 7)  ui_angle_deg -= 100.0f;
    else if (cursor_pos == 8)  ui_angle_deg -= 10.0f;
    else if (cursor_pos == 9)  ui_angle_deg -= 1.0f;
    else if (cursor_pos == 11) ui_angle_deg -= 0.1f;
    else if (cursor_pos == 13) rotation_cw = !rotation_cw;
  }
  // wrap angle 0..360
  while (ui_angle_deg >= 360.0f) ui_angle_deg -= 360.0f;
  while (ui_angle_deg < 0.0f)    ui_angle_deg += 360.0f;

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("   Angle Mode");
  lcd.setCursor(0, 1); lcd.print("Angle:  ");
  int ai = (int)ui_angle_deg;
  lcd.print((ai % 1000) / 100);
  lcd.print((ai % 100) / 10);
  lcd.print(ai % 10);
  lcd.print(".");
  lcd.print((int)((ui_angle_deg - floorf(ui_angle_deg)) * 10));
  lcd.setCursor(cursor_pos, 1);
}

void display_update(Button cmd) {
  if (cmd == BUTTON_SELECT) {
    if (current_screen == SCREEN_SELECT) {
      current_screen = selected_speed_mode ? SCREEN_SPEED : SCREEN_ANGLE;
      lock_cursor = !lock_cursor;
      if (lock_cursor) lcd.noCursor(); else lcd.cursor();
    } else {
      lock_cursor = !lock_cursor;
      if (lock_cursor) lcd.noCursor(); else lcd.cursor();
    }
  } else {
    if (current_screen == SCREEN_SPEED)      display_speed(cmd);
    else if (current_screen == SCREEN_ANGLE) display_angle(cmd);
    else                                     display_selection(cmd);
  }
  if (current_screen != SCREEN_SELECT) {
    lcd_show_direction(rotation_cw);
  }
}

// -----------------------------
// Main
// -----------------------------
void setup() {
  Serial.begin(115200);
  // LCD
  lcd_setup_screen();
  // Encoder pins
  pinMode(ENCODER_PINA, INPUT_PULLUP);
  pinMode(ENCODER_PINB, INPUT_PULLUP);
  pinMode(ENCODER_PINZ, INPUT_PULLUP);
  // Timer1 for timebase
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0; ovf_count = 0;
  TCCR1B |= (1 << CS11); // prescaler=8
  TIMSK1 = (1 << TOIE1); // overflow ISR
  // Pin change interrupts (falling) for A/B/Z
  attachPCINT(digitalPinToPCINT(ENCODER_PINA), isr_phase_a, FALLING);
  attachPCINT(digitalPinToPCINT(ENCODER_PINB), isr_phase_b, FALLING);
  attachPCINT(digitalPinToPCINT(ENCODER_PINZ), isr_phase_z, FALLING);
  // Stepper
  stepper_setup();
  stepper_enable(false);
}

static const unsigned long SPEED_CTRL_MS = 500;
static const unsigned long ANGLE_CTRL_MS = 500;
unsigned long last_ctrl_ms = 0;

void loop() {
  Button b = read_lcd_buttons();
  if (b != BUTTON_NULL) {
    display_update(b);
    stepper_set_direction(rotation_cw);
    if (current_screen == SCREEN_SELECT) {
      stepper_enable(false);
    } else if (current_screen == SCREEN_SPEED) {
      stepper_enable(true);
      // optional immediate speed set here; or keep periodic correction below
    } else if (current_screen == SCREEN_ANGLE) {
      stepper_enable(true);
      // angle mode will use periodic correction section below
    }
  }

  unsigned long now = millis();
  if (current_screen == SCREEN_ANGLE && (now - last_ctrl_ms) >= ANGLE_CTRL_MS) {
    last_ctrl_ms = now;
    float enc_angle = encoder_get_angle_deg();
    float err = ui_angle_deg - enc_angle;
    // normalize error to shortest path -180..180
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    float move = fabs(err);
    if (move > 1.0f) {
      // Generate pulses proportional to move; use a fixed frequency
      // 1.8 deg per full step for 200 steps/rev; simple proportional approximation
      // Number of steps needed ≈ move / 1.8
      float steps = move / (360.0f / STEPS_PER_REV);
      // Convert to a frequency for a short burst
      stepgen_set_hz(100.0f); // 100 steps/sec for gentle moves
      // This simple example does not implement counted pulses on Timer2 in this consolidated sketch
      // For precise angle moves, a counted-pulse approach (OCR2A ISR counting) should be used.
    }
  }

  if (current_screen == SCREEN_SPEED && (now - last_ctrl_ms) >= SPEED_CTRL_MS) {
    last_ctrl_ms = now;
    stepper_set_speed_rpm(ui_speed_rpm);
    Serial.print("Encoder RPM: "); Serial.print(encoder_get_rpm_from_z());
    Serial.print("  Set RPM: "); Serial.println(ui_speed_rpm);
  }
}
