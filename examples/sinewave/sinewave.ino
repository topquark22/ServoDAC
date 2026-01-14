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

const float MAX_FREQUENCY = 40.0f;

// frequency max skew per second
const float F_SKEW = 40.0f;

const float Vin = 5.0;
const float Vout = 5.0;

// LCD I2C address and size
const uint8_t LCD_ADDR = 0x27;
const uint8_t LCD_WIDTH = 16;
const uint8_t LCD_HEIGHT = 2;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT);

ServoDAC dac(PIN_CHARGE, PIN_DISCHARGE, PIN_FEEDBACK, R1, C1, R_D);

Dejitter pin(PIN_FREQUENCY, 5);
RateLimiter lim(F_SKEW, F_SKEW);

unsigned long start_ms;

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  pin.begin();
  dac.begin();

  start_ms = millis();

  int v_raw = pin.read();
  float v = v_raw * Vin / 1023.0f;
  lim.set_value(v);
  lim.begin();
}

float toFrequency(float voltage) {
  return MAX_FREQUENCY * (voltage / Vout);  // TODO make exponential
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

  int v_raw = pin.read();
  float v = adcToFloat(v_raw) * Vin;
  float t = (millis() - start_ms) * 1.0e-3f;
  float f_unlim = toFrequency(v);
  float f = lim.read(f_unlim);
  Serial.println(f);

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
