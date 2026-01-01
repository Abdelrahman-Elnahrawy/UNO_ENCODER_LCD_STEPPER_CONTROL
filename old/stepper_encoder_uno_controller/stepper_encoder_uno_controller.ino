/*
  Project: Stepper + Encoder Uno Controller
  Board:   Arduino Uno / Nano (ATmega328P)
  Author:  Abdelrahman Elnahrawy
  License: MIT

  Description:
  - LCD Keypad Shield UI (A0 buttons) to command Speed or Angle.
  - Rotary encoder (A/B/Z) feedback via PinChangeInterrupt and Timer1 timebase.
  - Stepper pulses generated on Timer2 OC2A (Pin 11). Angle moves support counted pulses.
  - Includes an initial homing-style move and incremental corrections.

  Fixes:
  - Defines STEP_ANGLE from STEPS_PER_REV (no undefined macros).
  - Parameterized ENCODER_PPR.
  - Timer2 uses OC2A only (no OC2B toggling).
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
static bool rotation_cw = true;
static bool lock_cursor = true;
static int cursor_pos = 7;
static bool selected_speed_mode = true;

// -----------------------------
// Encoder (A/B/Z)
// -----------------------------
static const uint8_t ENCODER_PINA = A1; // Phase A
static const uint8_t ENCODER_PINB = A2; // Phase B
static const uint8_t ENCODER_PINZ = A3; // Index Z (optional)
static const uint16_t ENCODER_PPR = 1500; // pulses per revolution

#define TICKS_TO_US(ticks) ((ticks) / 2)     // Timer1 @ prescaler=8 => 0.5us per tick
#define TICKS_TO_MS(ticks) ((ticks) / 2000.0)

volatile uint16_t enc_count_a = 0;
volatile uint16_t enc_count_b = 0;
volatile uint32_t last_ticks_a = 0, last_ticks_b = 0, last_ticks_z = 0;
volatile uint32_t pulse_ticks_z = 0;
volatile uint16_t ovf_count = 0;
volatile bool enc_dir_cw = true;

ISR(TIMER1_OVF_vect) { ovf_count++; }
static inline uint32_t ticks_now() { return (uint32_t(ovf_count) << 16) + TCNT1; }

void isr_phase_a() { uint32_t now = ticks_now(); enc_dir_cw = (digitalRead(ENCODER_PINB) == LOW); enc_count_a = enc_dir_cw ? (enc_count_a + 1) % ENCODER_PPR : (enc_count_a == 0 ? ENCODER_PPR - 1 : enc_count_a - 1); last_ticks_a = now; }
void isr_phase_b() { uint32_t now = ticks_now(); enc_dir_cw = (digitalRead(ENCODER_PINA) == HIGH); enc_count_b = enc_dir_cw ? (enc_count_b + 1) % ENCODER_PPR : (enc_count_b == 0 ? ENCODER_PPR - 1 : enc_count_b - 1); last_ticks_b = now; }
void isr_phase_z() { uint32_t now = ticks_now(); pulse_ticks_z = now - last_ticks_z; last_ticks_z = now; enc_count_a = 0; enc_count_b = 0; ovf_count = 0; TCNT1 = 0; }

float encoder_get_angle_deg() { float a_deg = (float)enc_count_a * (360.0f / ENCODER_PPR); float b_deg = (float)enc_count_b * (360.0f / ENCODER_PPR); return (a_deg + b_deg) * 0.5f; }
float encoder_get_rpm_from_z() { if (pulse_ticks_z == 0) return 0.0f; const float timer_hz = 2000000.0f; float rps = (timer_hz / (float)pulse_ticks_z); return rps * 60.0f; }

// -----------------------------
// Timer2 step generator (OC2A only)
// -----------------------------
static const uint8_t STEPPER_STEP_PIN = 11;    // OC2A toggles
static const uint8_t STEPPER_DIR_PIN  = 12;    // direction
static const uint8_t STEPPER_EN_PIN   = 13;    // enable, LOW=on

static uint8_t t2_prescaler_bits = 0;
static volatile uint16_t pulse_matches_target = 0;
static volatile uint16_t pulse_matches_count  = 0;

ISR(TIMER2_COMPA_vect) {
  if (pulse_matches_target == 0) return; // free-run
  pulse_matches_count++;
  if (pulse_matches_count >= pulse_matches_target) {
    TIMSK2 &= ~(1 << OCIE2A);
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
    pulse_matches_count = 0; pulse_matches_target = 0;
  }
}

void stepgen_init() { pinMode(STEPPER_STEP_PIN, OUTPUT); TCCR2A = 0; TCCR2B = 0; TCCR2A |= (1 << WGM21); TCCR2A |= (1 << COM2A0); }
void stepgen_stop() { TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20)); }

static void t2_apply_freq(float frequency_hz) {
  unsigned long top;
  if (frequency_hz >= 31373) { t2_prescaler_bits = (1 << CS20); top = (16000000UL / (1UL * 2UL * (unsigned long)frequency_hz)) - 1UL; }
  else if (frequency_hz >= 3921) { t2_prescaler_bits = (1 << CS21); top = (16000000UL / (8UL * 2UL * (unsigned long)frequency_hz)) - 1UL; }
  else if (frequency_hz >= 980)  { t2_prescaler_bits = (1 << CS21) | (1 << CS20); top = (16000000UL / (32UL * 2UL * (unsigned long)frequency_hz)) - 1UL; }
  else if (frequency_hz >= 490)  { t2_prescaler_bits = (1 << CS22); top = (16000000UL / (64UL * 2UL * (unsigned long)frequency_hz)) - 1UL; }
  else if (frequency_hz >= 245)  { t2_prescaler_bits = (1 << CS22) | (1 << CS20); top = (16000000UL / (128UL * 2UL * (unsigned long)frequency_hz)) - 1UL; }
  else if (frequency_hz >= 122)  { t2_prescaler_bits = (1 << CS22) | (1 << CS21); top = (16000000UL / (256UL * 2UL * (unsigned long)frequency_hz)) - 1UL; }
  else                           { t2_prescaler_bits = (1 << CS22) | (1 << CS21) | (1 << CS20); top = (16000000UL / (1024UL * 2UL * (unsigned long)frequency_hz)) - 1UL; }
  if (top > 255UL) top = 255UL; OCR2A = (uint8_t)top; TCCR2B = t2_prescaler_bits; TCCR2A = (1 << WGM21) | (1 << COM2A0);
}

void stepgen_set_speed_hz(float frequency_hz) { pulse_matches_target = 0; if (frequency_hz <= 0) { stepgen_stop(); return; } t2_apply_freq(frequency_hz); TIMSK2 &= ~(1 << OCIE2A); }
void stepgen_generate_steps(uint16_t steps, float frequency_hz) { if (steps == 0 || frequency_hz <= 0) return; stepgen_stop(); pulse_matches_count=0; pulse_matches_target=steps*2; TIMSK2 |= (1 << OCIE2A); t2_apply_freq(frequency_hz); }

void stepper_setup() { pinMode(STEPPER_DIR_PIN, OUTPUT); pinMode(STEPPER_EN_PIN, OUTPUT); digitalWrite(STEPPER_EN_PIN, HIGH); stepgen_init(); }
void stepper_enable(bool en) { digitalWrite(STEPPER_EN_PIN, en ? LOW : HIGH); if (!en) stepgen_stop(); }
void stepper_set_direction(bool cw) { digitalWrite(STEPPER_DIR_PIN, cw ? HIGH : LOW); }

static const uint16_t STEPS_PER_REV = 200; // 1.8 deg per step
static const float STEP_ANGLE_DEG = 360.0f / STEPS_PER_REV;

void stepper_set_speed_rpm(float rpm) { if (rpm <= 0) { stepgen_stop(); return; } float sps = (rpm * STEPS_PER_REV) / 60.0f; stepgen_set_speed_hz(sps); }
void stepper_move_degrees(float degrees, float rpm) { if (degrees <= 0 || rpm <= 0) return; uint16_t steps = (uint16_t)roundf(fabs(degrees) / STEP_ANGLE_DEG); float sps = (rpm * STEPS_PER_REV) / 60.0f; stepgen_generate_steps(steps, sps); }

// -----------------------------
// UI helpers
// -----------------------------
static const float MAX_SPEED_RPM = 3876.0f;
static const float SET_ANGLE_SPEED = 200.0f;
static float ui_angle_deg = 0.0f; static float ui_speed_rpm = 0.0f; static float move_angle_deg = 0.0f;

void lcd_setup_screen() { lcd.begin(16, 2); lcd.clear(); lcd.setCursor(0, 0); lcd.print(" CONTROL TYPE"); lcd.setCursor(0, 1); lcd.print("> SPEED    ANGLE "); }
void lcd_show_direction(bool cw) { lcd.setCursor(14, 1); lcd.print(cw ? "CW" : "CC"); lcd.setCursor(cursor_pos, 1); }

Button read_lcd_buttons() { if (analogRead(ADC_BUTTON_PIN) >= 1020) return BUTTON_NULL; delay(100); int adc = analogRead(ADC_BUTTON_PIN); if (adc<50) return BUTTON_RIGHT; else if (adc<200) return BUTTON_UP; else if (adc<380) return BUTTON_DOWN; else if (adc<600) return BUTTON_LEFT; else if (adc<800) return BUTTON_SELECT; int counter=0; while (analogRead(ADC_BUTTON_PIN)<1000) { counter++; delay(100); if (counter>20) { current_screen=SCREEN_SELECT; lcd.noCursor(); lock_cursor=true; return BUTTON_SELECT_HOLD; } if (counter>100){ lcd.clear(); lcd.setCursor(0,0); lcd.print("   Stuck button "); } } return BUTTON_NULL; }

void display_selection(Button cmd) { if (cmd==BUTTON_LEFT || cmd==BUTTON_RIGHT) selected_speed_mode = !selected_speed_mode; lcd.clear(); lcd.setCursor(0,0); lcd.print(" CONTROL TYPE"); lcd.setCursor(0,1); lcd.print(selected_speed_mode ? "> SPEED    ANGLE " : "  SPEED  > ANGLE "); }

void display_speed(Button cmd) {
  int min_cursor = 6;
  if (cmd==BUTTON_LEFT)  { cursor_pos--; if (cursor_pos==10 || cursor_pos==12) cursor_pos--; if (cursor_pos<min_cursor) cursor_pos=13; }
  if (cmd==BUTTON_RIGHT) { cursor_pos++; if (cursor_pos==10 || cursor_pos==12) cursor_pos++; if (cursor_pos>13) cursor_pos=min_cursor; }
  if (cmd==BUTTON_UP)    { if (cursor_pos==6) ui_speed_rpm+=1000; else if (cursor_pos==7) ui_speed_rpm+=100; else if (cursor_pos==8) ui_speed_rpm+=10; else if (cursor_pos==9) ui_speed_rpm+=1; else if (cursor_pos==11) ui_speed_rpm+=0.1; else if (cursor_pos==13) rotation_cw=!rotation_cw; }
  if (cmd==BUTTON_DOWN)  { if (cursor_pos==6) ui_speed_rpm-=1000; else if (cursor_pos==7) ui_speed_rpm-=100; else if (cursor_pos==8) ui_speed_rpm-=10; else if (cursor_pos==9) ui_speed_rpm-=1; else if (cursor_pos==11) ui_speed_rpm-=0.1; else if (cursor_pos==13) rotation_cw=!rotation_cw; }
  if (ui_speed_rpm < 0) ui_speed_rpm = 0; if (ui_speed_rpm > MAX_SPEED_RPM) ui_speed_rpm = MAX_SPEED_RPM;
  lcd.clear(); lcd.setCursor(0,0); lcd.print("   Speed Mode"); lcd.setCursor(0,1); lcd.print("Speed: "); int si=(int)ui_speed_rpm; lcd.print(si>999?si/1000:0); lcd.print((si%1000)/100); lcd.print((si%100)/10); lcd.print(si%10); lcd.print("."); lcd.print((int)((ui_speed_rpm - floorf(ui_speed_rpm))*10)); lcd.setCursor(cursor_pos, 1);
}

void display_angle(Button cmd) {
  int min_cursor = 7;
  if (cmd==BUTTON_LEFT)  { cursor_pos--; if (cursor_pos==10 || cursor_pos==12) cursor_pos--; if (cursor_pos<min_cursor) cursor_pos=13; }
  if (cmd==BUTTON_RIGHT) { cursor_pos++; if (cursor_pos==10 || cursor_pos==12) cursor_pos++; if (cursor_pos>13) cursor_pos=min_cursor; }
  if (cmd==BUTTON_UP)    { if (cursor_pos==7) ui_angle_deg+=100; else if (cursor_pos==8) ui_angle_deg+=10; else if (cursor_pos==9) ui_angle_deg+=1; else if (cursor_pos==11) ui_angle_deg+=0.1; else if (cursor_pos==13) rotation_cw=!rotation_cw; }
  if (cmd==BUTTON_DOWN)  { if (cursor_pos==7) ui_angle_deg-=100; else if (cursor_pos==8) ui_angle_deg-=10; else if (cursor_pos==9) ui_angle_deg-=1; else if (cursor_pos==11) ui_angle_deg-=0.1; else if (cursor_pos==13) rotation_cw=!rotation_cw; }
  while (ui_angle_deg >= 360.0f) ui_angle_deg -= 360.0f; while (ui_angle_deg < 0.0f) ui_angle_deg += 360.0f;
  lcd.clear(); lcd.setCursor(0,0); lcd.print("   Angle Mode"); lcd.setCursor(0,1); lcd.print("Angle:  "); int ai=(int)ui_angle_deg; lcd.print((ai%1000)/100); lcd.print((ai%100)/10); lcd.print(ai%10); lcd.print("."); lcd.print((int)((ui_angle_deg - floorf(ui_angle_deg))*10)); lcd.setCursor(cursor_pos, 1);
}

void display_update(Button cmd) { if (cmd==BUTTON_SELECT) { if (current_screen==SCREEN_SELECT) current_screen=(selected_speed_mode?SCREEN_SPEED:SCREEN_ANGLE); lock_cursor=!lock_cursor; if (lock_cursor) lcd.noCursor(); else lcd.cursor(); } else { if (current_screen==SCREEN_SPEED) display_speed(cmd); else if (current_screen==SCREEN_ANGLE) display_angle(cmd); else display_selection(cmd); } if (current_screen!=SCREEN_SELECT) lcd_show_direction(rotation_cw); }

// -----------------------------
// Setup and loop
// -----------------------------
static const float HOMING_SPEED_RPM = 60.0f;
static const unsigned long SPEED_CTRL_MS = 500; static const unsigned long ANGLE_CTRL_MS = 50; unsigned long last_ctrl_ms = 0;

void setup() {
  Serial.begin(115200);
  lcd_setup_screen();
  // Encoder pins and Timer1
  pinMode(ENCODER_PINA, INPUT_PULLUP); pinMode(ENCODER_PINB, INPUT_PULLUP); pinMode(ENCODER_PINZ, INPUT_PULLUP);
  TCCR1A=0; TCCR1B=0; TCNT1=0; ovf_count=0; TIMSK1=(1<<TOIE1); TCCR1B|=(1<<CS11);
  attachPCINT(digitalPinToPCINT(ENCODER_PINA), isr_phase_a, FALLING);
  attachPCINT(digitalPinToPCINT(ENCODER_PINB), isr_phase_b, FALLING);
  attachPCINT(digitalPinToPCINT(ENCODER_PINZ), isr_phase_z, FALLING);
  // Stepper
  stepper_setup(); stepper_enable(true);

  // Initial move: one full revolution
  stepper_set_direction(true); stepper_move_degrees(360.0f, HOMING_SPEED_RPM); delay(5000);
  // Nudge towards small angle
  while (encoder_get_angle_deg() > 2.0f) { stepper_move_degrees(1.8f, HOMING_SPEED_RPM); delay(50); }
  stepper_enable(false);
}

void loop() {
  Button b = read_lcd_buttons();
  if (b != BUTTON_NULL) {
    display_update(b);
    stepper_set_direction(rotation_cw);
    if (current_screen == SCREEN_SELECT) stepper_enable(false); else stepper_enable(true);
    if (current_screen == SCREEN_ANGLE && b == BUTTON_SELECT) { float enc = encoder_get_angle_deg(); move_angle_deg = fmod(enc + ui_angle_deg, 360.0f); }
  }

  unsigned long now = millis();

  if (current_screen == SCREEN_ANGLE && (now - last_ctrl_ms) >= ANGLE_CTRL_MS) {
    last_ctrl_ms = now;
    float enc = encoder_get_angle_deg(); float err = move_angle_deg - enc; while (err > 180.0f) err -= 360.0f; while (err < -180.0f) err += 360.0f;
    float mag = fabs(err);
    if (mag > (STEP_ANGLE_DEG * 2)) { stepper_move_degrees(mag * 0.5f, SET_ANGLE_SPEED); }
    else if (mag > STEP_ANGLE_DEG)  { stepper_move_degrees(mag, SET_ANGLE_SPEED); }
  }

  if (current_screen == SCREEN_SPEED && (now - last_ctrl_ms) >= SPEED_CTRL_MS) {
    last_ctrl_ms = now;
    float act = encoder_get_rpm_from_z(); float tgt = ui_speed_rpm; stepper_set_speed_rpm(tgt);
    Serial.print("Actual RPM:"); Serial.print(act); Serial.print("  Set RPM:"); Serial.println(tgt);
  }
}
