
#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
#include "servodac.h"

// --- wiring pins (matching the original sketch defaults) ---
const uint8_t PIN_CHARGE     = 5;   // charge pin (pulse high, then hi-Z)
const uint8_t PIN_DISCHARGE  = 4;   // discharge pin (active-high)
const uint8_t PIN_FEEDBACK   = A2;  // feedback pin

// R1 = 1K
// C11 = 1uf
const float TAU = 1.0e-3; // time constant R1 * C1 (seconds)
const float RD = 1000;    // discharge resistor (ohms)

const uint8_t PIN_FREQUENCY = A3;

const float Vin = 5.0;
const float Vout = 5.0;

// LCD I2C address and size
const uint8_t LCD_I2C_ADDR = 0x27;
const uint8_t LCD_WIDTH = 16;
const uint8_t LCD_HEIGHT = 2;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_WIDTH, LCD_HEIGHT);

ServoDAC dac(PIN_CHARGE, PIN_DISCHARGE, PIN_FEEDBACK, TAU, RD);

unsigned long start_us;

void setup() {
  // target input pin
  pinMode(PIN_FREQUENCY, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  dac.begin();
  start_us = micros();
}

float toFrequency(float voltage) {
  return 20.0 * (voltage / Vout); // TODO make exponential
}

float t1 = 0;
float f1 = 0;

void updateLCD(float freq) {
  lcd.clear();
  lcd.print(F("f="));
  lcd.print(freq);
}

int prev_v_raw = -1;

float f0 = 0;
float t0 = 0;

void loop() {
  static unsigned long next_us = 0;
  if (next_us == 0) next_us = micros();

  int v_raw = analogRead(PIN_FREQUENCY);
  float v = ServoDAC::adcToVoltage(v_raw);

  float t = (micros() - start_us) * 1.0e-6f;
  float f = toFrequency(v);

  if (f == 0) {
    dac.update(Vout / 2);

  } else {
    float t1 = t0 - (f0 / f) * (t - t0);
    float y = sin(2 * PI * f * (t - t1));

    float v = (y + 1) * (Vout / 2);
    dac.update(v);

  }

  if (v_raw != prev_v_raw) {
    updateLCD(f);
    prev_v_raw = v_raw;
  }

  t0 = t1;
  f0 = f;
}
