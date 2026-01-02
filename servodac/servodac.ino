#include "LiquidCrystal_I2C.h"

#include <limits.h>
#include <math.h>   // for logf

// Small epsilon to avoid divide-by-zero and proof against ADC noise near rails
static const float EPS_V = 0.001f;      // 1 mV
static const float EPS_RATIO = 1e-6f;   // dimensionless

// Constants (set these based on your circuit)
const float C = 1e-6;      // C1
const float Rc = 1e3;       // charging resistor
const float Rd = 1e3;       // discharging resistor
const float V_IN = 5.0;     // Pulse voltage (typically 5V)

const float deadband = 0.02; // volts

const unsigned long MAX_CHARGE_PULSE = 12000;
const unsigned long MAX_DISCHARGE_PULSE = 12000;

// How often to update the output voltage (ms).
// Since for output +5v takes 7 ms, this should be larger than that.
const unsigned long UPDATE_INTERVAL_MS = 10;
// Update the LCD each this many frames
const unsigned int LCD_RATE = 10;

// pin for charging cap
const PROGMEM uint8_t PIN_OUT = 5;

// kill output to 0 via 2N7000 MOSFET
const PROGMEM uint8_t PIN_RESET = 4;

// pin for injecting test voltage
const uint8_t PIN_TEST_IN = A3;
// pin for sampling circuit output
const uint8_t PIN_CIRCUIT_IN = A2;

const int BAUD_RATE = 9600;
// LCD I2C address and size

const PROGMEM uint8_t LCD_I2C_ADDR = 0x27;
const PROGMEM uint8_t LCD_WIDTH = 16;
const PROGMEM uint8_t LCD_HEIGHT = 2;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_WIDTH, LCD_HEIGHT);

// Calculate charging time (µs) to move C1 from sample to target
// Assumes target > sample
unsigned int calcChargePulse(float target, float sample) {
  if (target <= sample) return 0;

  // Clamp target/sample to sane range
  if (target > V_IN - EPS_V) target = V_IN - EPS_V;
  if (sample > V_IN - EPS_V) sample = V_IN - EPS_V;

  // If sample is already basically at the rail, you can't charge higher meaningfully
  if ((V_IN - sample) <= EPS_V) return 0;

  const float tau = Rc * C;  // seconds

  float denom = (V_IN - sample);
  float numer = (V_IN - target);

  float ratio = numer / denom;

  // ratio should be in (0, 1). If it's <=0, we'd need infinite time -> saturate.
  if (ratio <= EPS_RATIO) {
    return MAX_CHARGE_PULSE;
  }
  if (ratio >= 1.0f) return 0; // target ~= sample

  float t_sec = -tau * logf(ratio);
  float t_us_f = t_sec * 1e6f;

  if (t_us_f <= 0.0f) return 0;
  if (t_us_f > (float)MAX_CHARGE_PULSE) return MAX_CHARGE_PULSE;

  return (unsigned int)(t_us_f + 0.5f); // round
}


// Calculate discharge time (µs) to move C1 from sample down to target
// Assumes target < sample
unsigned int calcDischargePulse(float target, float sample) {
  if (target >= sample) return 0;

  // Clamp away from 0 to avoid ratio=0
  if (target < EPS_V) target = EPS_V;
  if (sample < EPS_V) sample = EPS_V;

  const float tau = Rd * C;  // seconds

  float ratio = target / sample;

  // ratio should be in (0, 1). If <=0, infinite time -> saturate.
  if (ratio <= EPS_RATIO) return MAX_DISCHARGE_PULSE;
  if (ratio >= 1.0f) return 0;

  float t_sec = -tau * logf(ratio);
  float t_us_f = t_sec * 1e6f;

  if (t_us_f <= 0.0f) return 0;
  if (t_us_f > (float)MAX_DISCHARGE_PULSE) return MAX_DISCHARGE_PULSE;

  return (unsigned int)(t_us_f + 0.5f); // round
}

void setup() {
  
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_TEST_IN, INPUT);
  pinMode(PIN_CIRCUIT_IN, INPUT);
  pinMode(PIN_OUT, INPUT); // high-impedance
  digitalWrite(PIN_OUT, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  Serial.begin(BAUD_RATE);
}

void charge(unsigned long pulse) {
  pinMode(PIN_OUT, OUTPUT);
  digitalWrite(PIN_OUT, HIGH);
  delayMicroseconds(pulse);
  pinMode(PIN_OUT, INPUT); // high-impedance
  digitalWrite(PIN_OUT, LOW);
}

void discharge(unsigned long pulse) {
  digitalWrite(PIN_RESET, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(PIN_RESET, LOW);
}

float toVoltage(int raw) {
  return raw * V_IN / 1024;
}

void updateLCD(float target, float sample, float error, unsigned long pulse) {
  // target voltage
  lcd.setCursor(0, 0);
  lcd.print(target);
  lcd.print(F("V  "));

  // charging time
  lcd.setCursor(0, 1);
  lcd.print(pulse);
  lcd.print(F("us  "));

  // actual output voltage
  lcd.setCursor(8, 0);
  lcd.print(sample);
  lcd.print(F("V  "));
 
  // dhow much charge C1 lost during the frame
  lcd.setCursor(8, 1);
  lcd.print(error);
  lcd.print(F("V  "));
}

unsigned int loopCt = 0;

void loop() {
  static unsigned long next_us = 0;
  if (next_us == 0) next_us = micros();

  // --- sample ---
  int sample_raw = analogRead(PIN_CIRCUIT_IN);
  float sample = toVoltage(sample_raw);

  int target_raw = analogRead(PIN_TEST_IN);
  float target = toVoltage(target_raw);

  float error = target - sample;

  unsigned int pulse = 0;

  if (fabs(error) >= deadband) {
    if (error > 0) {
      pulse = calcChargePulse(target, sample);
      pulse = min(pulse, MAX_CHARGE_PULSE);
      charge(pulse);
    } else {
      pulse = calcDischargePulse(target, sample);
      pulse = min(pulse, MAX_DISCHARGE_PULSE);
      discharge(pulse);
    }
  }

  if (0 == loopCt) {
    updateLCD(target, sample, error, pulse);
  }
  loopCt = (loopCt + 1) % LCD_RATE;

  // --- wait until next frame boundary ---
  next_us += (unsigned long)UPDATE_INTERVAL_MS * 1000UL;
  while ((long)(micros() - next_us) < 0) {
    // optionally yield() or do nothing
  }
}
