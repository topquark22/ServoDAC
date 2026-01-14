#include <math.h>

#include <LiquidCrystal_I2C.h>
#include <GPIOUtils.h>

#include "ServoDAC.h"

// --- wiring pins (matching the original sketch defaults) ---
const uint8_t PIN_CHARGE = 5;     // charge pin (pulse high, then hi-Z)
const uint8_t PIN_DISCHARGE = 4;  // discharge pin (active-high)
const uint8_t PIN_FEEDBACK = A2;  // feedback pin

const uint8_t PIN_FREQUENCY = A3;  // connect to voltage divider

// R1 = 2.2K
// C1 = 470nf
// R_D = 1.0k
const float R1 = 2.2e3;  // Charging resistor (ohms)
const float C1 = 470e-9; // Integrating capacitor (farads)
const float R_D = 2.2e3;  // discharge resistor (ohms)

const float MIN_FREQUENCY = 1.0f;
const float MAX_FREQUENCY = 40.0f;

const float Vin = 5.0;
const float Vout = 5.0;

// LCD I2C address and size
const uint8_t LCD_ADDR = 0x27;
const uint8_t LCD_WIDTH = 16;
const uint8_t LCD_HEIGHT = 2;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT);

ServoDAC dac(PIN_CHARGE, PIN_DISCHARGE, PIN_FEEDBACK, R1, C1, R_D);
Dejitter pin(PIN_FREQUENCY, 2);

static unsigned long start_ms;

inline int sgn(float x) {
  return (x > 0) - (x < 0);
}

float toFrequency(float potVoltage) {
  return MAX_FREQUENCY * (potVoltage / Vout);  // TODO make exponential
}

static float f0 = 0.0f;

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  pin.begin();
  dac.begin();

  start_ms = millis();

  int pot = pin.read();
  float potVoltage = adcToFloat(pot, 0.0f, Vin);
  f0 = toFrequency(potVoltage);
}


void updateLCD(float freq) {
  lcd.clear();
  lcd.print(F("f="));
  lcd.print(freq);
}

float yToVoltage(float y) {
  return((y + 1) * (Vout / 2));
}

void loop() {
  static float phaseTime = 0.0f;  // seconds into the cycle, wrapped
  static float t0 = 0.0f;

  int pot = pin.read();
  float potVoltage = adcToFloat(pot, 0.0f, Vin);
  float t = (millis() - start_ms) * 1.0e-3f;

  float dt = t - t0;
  t0 = t;

  float f = toFrequency(potVoltage);

  if (fabsf(f) < 1.0e-3f) {
    dac.update(yToVoltage(0.0f));
    phaseTime = 0.0f;           // optional: reset when stopped
  } else {
    phaseTime += dt;

    float T = 1.0f / f;         // period
    phaseTime = fmodf(phaseTime, T);
    if (phaseTime < 0) phaseTime += T;

    float y = sinf(2.0f * PI * f * phaseTime);
    dac.update(yToVoltage(y));
  }

  // LCD update only on pot change
  static int prev_pot = -1024;
  if (pot != prev_pot) {
    updateLCD(f);
    prev_pot = pot;
  }
}
