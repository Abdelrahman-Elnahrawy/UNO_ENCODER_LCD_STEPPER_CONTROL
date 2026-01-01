/*
  Project: LCD Keypad Stepper Closed-Loop Controller
  Board:   Arduino Uno / Nano (ATmega328P)
  Author:  Abdelrahman Elnahrawy
  License: MIT

  Description:
  - LCD Keypad Shield UI (A0 buttons) to command Speed or Angle.
  - Rotary encoder (A/B/Z) feedback via PinChangeInterrupt and Timer1 timebase.
  - Stepper pulses generated on Timer2 OC2A (Pin 11). Angle moves use counted pulses.
  - Closed-loop corrections performed periodically for both Speed and Angle modes.

  Key fixes compared to older versions:
  - Encoder wraps parameterized by ENCODER_PPR; no hardcoded 1500.
  - Timer2 only toggles OC2A (Pin 11). OC2B (Pin 3) is not used.
  - Clear constants and comments; structured sections for maintainability.
*/

#include <Arduino.h>
#include <LiquidCrystal.h>
#include "PinChangeInterrupt.h"

// -----------------------------
// LCD Keypad shield
// -----------------------------
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
static const uint8_t ADC_BUTTON_PIN = A0; // analog ladder

enum Button : uint8_t { BUTTON_NULL=0, BUTTON_RIGHT, BUTTON_LEFT, BUTTON_UP, BUTTON_DOWN, BUTTON_SELECT, BUTTON_SELECT_HOLD };

enum ScreenMode : uint8_t { SCREEN_ANGLE=0, SCREEN_SPEED=1, SCREEN_SELECT=2 };
static ScreenMode current_screen = SCREEN_SELECT;
static bool rotation_cw = true; // CW=true, CCW=false
static bool lock_cursor = true; // SELECT toggles cursor lock
static int cursor_pos = 7;      // editable field position
static bool selected_speed_mode = true;

// -----------------------------
// Encoder (A/B/Z) with pin change interrupts
// -----------------------------
static const uint8_t ENCODER_PINA = A1; // Phase A
static const uint8_t ENCODER_PINB = A2; // Phase B
static const uint8_t ENCODER_PINZ = A3; // Index Z (optional)
static const uint16_t ENCODER_PPR = 1500; // pulses per revolution for your encoder

#define TICKS_TO_US(ticks) ((ticks) / 2)     // Timer1 @ prescaler=8 => 2 MHz => 0.5us per tick
#define TICKS_TO_MS(ticks) ((ticks) / 2000.0)

volatile uint16_t enc_count_a = 0;
volatile uint16_t enc_count_b = 0;
volatile uint32_t last_ticks_a = 0, last_ticks_b = 0, last_ticks_z = 0;
volatile uint32_t pulse_ticks_z = 0;
volatile uint16_t ovf_count = 0;
volatile bool enc_dir_cw = true; // true=CW, false=CCW

ISR(TIMER1_OVF_vect) { ovf_count++; }

static inline uint32_t ticks_now() { return (uint32_t(ovf_count) << 16) + TCNT1; }

void isr_phase_a() {
  uint32_t now = ticks_now();
  enc_dir_cw = (digitalRead(ENCODER_PINB) == LOW); // falling on A, if B low => CW
  if (enc_dir_cw) enc_count_a = (enc_count_a + 1) % ENCODER_PPR;
  else            enc_count_a = (enc_count_a == 0) ? (ENCODER_PPR - 1) : (enc_count_a - 1);
  last_ticks_a = now;
}

void isr_phase_b() {
  uint32_t now = ticks_now();
  enc_dir_cw = (digitalRead(ENCODER_PINA) == HIGH); // falling on B, if A high => CW
  if (enc_dir_cw) enc_count_b = (enc_count_b + 1) % ENCODER_PPR;
  else            enc_count_b = (enc_count_b == 0) ? (ENCODER_PPR - 1) : (enc_count_b - 1);
  last_ticks_b = now;
}

void isr_phase_z() {
  uint32_t now = ticks_now();
  pulse_ticks_z = now - last_ticks_z;
  last_ticks_z = now;
  enc_count_a = 0; enc_count_b = 0; ovf_count = 0; TCNT1 = 0;
}

float encoder_get_angle_deg() {
  float a_deg = (float)enc_count_a * (360.0f / ENCODER_PPR);
  float b_deg = (float)enc_count_b * (360.0f / ENCODER_PPR);
  return (a_deg + b_deg) * 0.5f;
}

float encoder_get_rpm_from_z() {
  if (pulse_ticks_z == 0) return 0.0f;
  const float timer_hz = 2000000.0f; // Timer1 at 2 MHz
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
static volatile uint16_t pulse_matches_target = 0; // target COMPA matches (two matches per step)
static volatile uint16_t pulse_matches_count  = 0;

ISR(TIMER2_COMPA_vect) {
  if (pulse_matches_target == 0) return; // free-run mode (speed control)
  pulse_matches_count++;
  if (pulse_matches_count >= pulse_matches_target) {
    // stop counted pulse burst
    TIMSK2 &= ~(1 << OCIE2A);
    // keep timer running only if used for speed mode; here we stop
    // safer to disable to ensure no extra edges
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
    pulse_matches_count = 0;
    pulse_matches_target = 0;
  }
}

void stepgen_init() {
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  // Timer2 CTC, toggle OC2A, stop timer initially
  TCCR2A = 0; TCCR2B = 0;
  TCCR2A |= (1 << WGM21);  // CTC mode
  TCCR2A |= (1 << COM2A0); // toggle OC2A on match
}

void stepgen_stop() {
  // Stop Timer2 by clearing prescaler bits
  TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
}

static void t2_apply_freq(float frequency_hz) {
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
  TCCR2A = (1 << WGM21) | (1 << COM2A0);
}

void stepgen_set_speed_hz(float frequency_hz) {
  pulse_matches_target = 0; // ensure free-run
  if (frequency_hz <= 0) { stepgen_stop(); return; }
  t2_apply_freq(frequency_hz);
  // No ISR used in free-run speed mode
  TIMSK2 &= ~(1 << OCIE2A);
}

void stepgen_generate_steps(uint16_t steps, float frequency_hz) {
  if (steps == 0 || frequency_hz <= 0) return;
  stepgen_stop();
  pulse_matches_count  = 0;
  pulse_matches_target = steps * 2; // two compare matches per STEP rising edge
  // Enable COMPA interrupt for counted pulses
  TIMSK2 |= (1 << OCIE2A);
  t2_apply_freq(frequency_hz);
}

void stepper_setup() {
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, HIGH); // disabled by default
  stepgen_init();
}

void stepper_enable(bool en) { digitalWrite(STEPPER_EN_PIN, en ? LOW : HIGH); if (!en) stepgen_stop(); }
void stepper_set_direction(bool cw) { digitalWrite(STEPPER_DIR_PIN, cw ? HIGH : LOW); }

static const uint16_t STEPS_PER_REV = 200;              // motor full steps per rev (e.g., 200 => 1.8 deg)
static const float    STEP_ANGLE_DEG = 360.0f / STEPS_PER_REV;

void stepper_set_speed_rpm(float rpm) {
  if (rpm <= 0) { stepgen_stop(); return; }
  float steps_per_s = (rpm * STEPS_PER_REV) / 60.0f;
  stepgen_set_speed_hz(steps_per_s);
}

void stepper_move_degrees(float degrees, float rpm) {
  if (degrees <= 0 || rpm <= 0) return;
  uint16_t steps = (uint16_t)roundf(fabs(degrees) / STEP_ANGLE_DEG);
  float steps_per_s = (rpm * STEPS_PER_REV) / 60.0f;
  stepgen_generate_steps(steps, steps_per_s);
}

// -----------------------------
// UI helpers
// -----------------------------
static const float MAX_SPEED_RPM = 3876.0f;
static float ui_angle_deg = 0.0f;
static float ui_speed_rpm = 0.0f;

void lcd_setup_screen() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(" CONTROL TYPE");
  lcd.setCursor(0, 1); lcd.print("> SPEED    ANGLE ");
}

void lcd_show_direction(bool cw) { lcd.setCursor(14, 1); lcd.print(cw ? "CW" : "CC"); lcd.setCursor(cursor_pos, 1); }

Button read_lcd_buttons() {
  if (analogRead(ADC_BUTTON_PIN) >= 1020) return BUTTON_NULL;
  delay(100);
  int adc = analogRead(ADC_BUTTON_PIN);
  if      (adc <  50) return BUTTON_RIGHT;
  else if (adc < 200) return BUTTON_UP;
  else if (adc < 380) return BUTTON_DOWN;
  else if (adc < 600) return BUTTON_LEFT;
  else if (adc < 800) return BUTTON_SELECT;
  int counter = 0;
  while (analogRead(ADC_BUTTON_PIN) < 1000) {
    counter++; delay(100);
    if (counter > 20) { current_screen = SCREEN_SELECT; lcd.noCursor(); lock_cursor = true; return BUTTON_SELECT_HOLD; }
    if (counter > 100) { lcd.clear(); lcd.setCursor(0, 0); lcd.print("   Stuck button "); }
  }
  return BUTTON_NULL;
}

void display_selection(Button cmd) {
  if (cmd == BUTTON_LEFT || cmd == BUTTON_RIGHT) selected_speed_mode = !selected_speed_mode;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(" CONTROL TYPE");
  lcd.setCursor(0, 1); lcd.print(selected_speed_mode ? "> SPEED    ANGLE " : "  SPEED  > ANGLE ");
}

void display_speed(Button cmd) {
  int min_cursor = 6;
  if (cmd == BUTTON_LEFT)  { cursor_pos--; if (cursor_pos == 10 || cursor_pos == 12) cursor_pos--; if (cursor_pos < min_cursor) cursor_pos = 13; }
  if (cmd == BUTTON_RIGHT) { cursor_pos++; if (cursor_pos == 10 || cursor_pos == 12) cursor_pos++; if (cursor_pos > 13) cursor_pos = min_cursor; }
  if (cmd == BUTTON_UP)    { if      (cursor_pos == 6)  ui_speed_rpm += 1000.0f; else if (cursor_pos == 7)  ui_speed_rpm += 100.0f; else if (cursor_pos == 8)  ui_speed_rpm += 10.0f; else if (cursor_pos == 9)  ui_speed_rpm += 1.0f; else if (cursor_pos == 11) ui_speed_rpm += 0.1f; else if (cursor_pos == 13) rotation_cw = !rotation_cw; }
  if (cmd == BUTTON_DOWN)  { if      (cursor_pos == 6)  ui_speed_rpm -= 1000.0f; else if (cursor_pos == 7)  ui_speed_rpm -= 100.0f; else if (cursor_pos == 8)  ui_speed_rpm -= 10.0f; else if (cursor_pos == 9)  ui_speed_rpm -= 1.0f; else if (cursor_pos == 11) ui_speed_rpm -= 0.1f; else if (cursor_pos == 13) rotation_cw = !rotation_cw; }
  if (ui_speed_rpm < 0) ui_speed_rpm = 0; if (ui_speed_rpm > MAX_SPEED_RPM) ui_speed_rpm = MAX_SPEED_RPM;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("   Speed Mode");
  lcd.setCursor(0, 1); lcd.print("Speed: ");
  int si = (int)ui_speed_rpm;
  lcd.print(si > 999 ? si / 1000 : 0);
  lcd.print((si % 1000) / 100); lcd.print((si % 100) / 10); lcd.print(si % 10);
  lcd.print("."); lcd.print((int)((ui_speed_rpm - floorf(ui_speed_rpm)) * 10));
  lcd.setCursor(cursor_pos, 1);
}

void display_angle(Button cmd) {
  int min_cursor = 7;
  if (cmd == BUTTON_LEFT)  { cursor_pos--; if (cursor_pos == 10 || cursor_pos == 12) cursor_pos--; if (cursor_pos < min_cursor) cursor_pos = 13; }
  if (cmd == BUTTON_RIGHT) { cursor_pos++; if (cursor_pos == 10 || cursor_pos == 12) cursor_pos++; if (cursor_pos > 13) cursor_pos = min_cursor; }
  if (cmd == BUTTON_UP)    { if      (cursor_pos == 7)  ui_angle_deg += 100.0f; else if (cursor_pos == 8)  ui_angle_deg += 10.0f; else if (cursor_pos == 9)  ui_angle_deg += 1.0f; else if (cursor_pos == 11) ui_angle_deg += 0.1f; else if (cursor_pos == 13) rotation_cw = !rotation_cw; }
  if (cmd == BUTTON_DOWN)  { if      (cursor_pos == 7)  ui_angle_deg -= 100.0f; else if (cursor_pos == 8)  ui_angle_deg -= 10.0f; else if (cursor_pos == 9)  ui_angle_deg -= 1.0f; else if (cursor_pos == 11) ui_angle_deg -= 0.1f; else if (cursor_pos == 13) rotation_cw = !rotation_cw; }
  while (ui_angle_deg >= 360.0f) ui_angle_deg -= 360.0f; while (ui_angle_deg < 0.0f) ui_angle_deg += 360.0f;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("   Angle Mode");
  lcd.setCursor(0, 1); lcd.print("Angle:  ");
  int ai = (int)ui_angle_deg;
  lcd.print((ai % 1000) / 100); lcd.print((ai % 100) / 10); lcd.print(ai % 10);
  lcd.print("."); lcd.print((int)((ui_angle_deg - floorf(ui_angle_deg)) * 10));
  lcd.setCursor(cursor_pos, 1);
}

void display_update(Button cmd) {
  if (cmd == BUTTON_SELECT) {
    if (current_screen == SCREEN_SELECT) { current_screen = selected_speed_mode ? SCREEN_SPEED : SCREEN_ANGLE; }
    lock_cursor = !lock_cursor; if (lock_cursor) lcd.noCursor(); else lcd.cursor();
  } else {
    if      (current_screen == SCREEN_SPEED)  display_speed(cmd);
    else if (current_screen == SCREEN_ANGLE)  display_angle(cmd);
    else                                      display_selection(cmd);
  }
  if (current_screen != SCREEN_SELECT) lcd_show_direction(rotation_cw);
}

// -----------------------------
// Control loops
// -----------------------------
static const unsigned long SPEED_CTRL_MS = 500;
static const unsigned long ANGLE_CTRL_MS = 100;
unsigned long last_ctrl_ms = 0;

// Simple PID for speed (optional)
float rpmKp = 0.2f, rpmKi = 0.05f, rpmKd = 0.32f;
float rpmIntegral = 0.0f, rpmPrevError = 0.0f;

void setup() {
  Serial.begin(115200);
  // LCD
  lcd_setup_screen();
  // Encoder pins
  pinMode(ENCODER_PINA, INPUT_PULLUP);
  pinMode(ENCODER_PINB, INPUT_PULLUP);
  pinMode(ENCODER_PINZ, INPUT_PULLUP);
  // Timer1 timebase
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0; ovf_count = 0; TIMSK1 = (1 << TOIE1); TCCR1B |= (1 << CS11);
  // Pin change interrupts
  attachPCINT(digitalPinToPCINT(ENCODER_PINA), isr_phase_a, FALLING);
  attachPCINT(digitalPinToPCINT(ENCODER_PINB), isr_phase_b, FALLING);
  attachPCINT(digitalPinToPCINT(ENCODER_PINZ), isr_phase_z, FALLING);
  // Stepper
  stepper_setup();
  stepper_enable(false);
}

void loop() {
  Button b = read_lcd_buttons();
  if (b != BUTTON_NULL) {
    display_update(b);
    stepper_set_direction(rotation_cw);
    if (current_screen == SCREEN_SELECT) stepper_enable(false);
    else stepper_enable(true);
  }

  unsigned long now = millis();

  if (current_screen == SCREEN_ANGLE && (now - last_ctrl_ms) >= ANGLE_CTRL_MS) {
    last_ctrl_ms = now;
    float enc_angle = encoder_get_angle_deg();
    float err = ui_angle_deg - enc_angle;
    while (err > 180.0f)  err -= 360.0f; // shortest path
    while (err < -180.0f) err += 360.0f;
    float move_deg = fabs(err);
    if (move_deg > (STEP_ANGLE_DEG * 0.5f)) {
      // Set direction per sign of error
      stepper_set_direction(err >= 0.0f);
      // Convert degrees to steps and generate counted pulses at gentle speed
      uint16_t steps = (uint16_t)roundf(move_deg / STEP_ANGLE_DEG);
      float steps_per_s = 100.0f; // gentle move
      stepgen_generate_steps(steps, steps_per_s);
    }
  }

  if (current_screen == SCREEN_SPEED && (now - last_ctrl_ms) >= SPEED_CTRL_MS) {
    last_ctrl_ms = now;
    float actual_rpm = encoder_get_rpm_from_z();
    float target_rpm = ui_speed_rpm;
    float err = target_rpm - actual_rpm;
    rpmIntegral += err;
    float deriv = err - rpmPrevError;
    float adj = rpmKp*err + rpmKi*rpmIntegral + rpmKd*deriv;
    rpmPrevError = err;
    float cmd_rpm = target_rpm + adj; if (cmd_rpm < 0) cmd_rpm = 0;
    stepper_set_speed_rpm(cmd_rpm);
    Serial.print("RPM act:"); Serial.print(actual_rpm); Serial.print("  tgt:"); Serial.print(target_rpm); Serial.print("  cmd:"); Serial.println(cmd_rpm);
  }
}
