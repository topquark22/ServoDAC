#include <math.h>

#include <LiquidCrystal_I2C.h>

#include "ServoDAC.h"

// --- wiring pins (matching the original sketch defaults) ---
const uint8_t PIN_CHARGE = 5;     // charge pin (pulse high, then hi-Z)
const uint8_t PIN_DISCHARGE = 4;  // discharge pin (active-high)
const uint8_t PIN_FEEDBACK = A2;  // feedback pin

const uint8_t PIN_FREQUENCY = A3;  // connect to voltage divider

// R1 = 2.2K
// C1 = 470nf
const float TAU = 1.0e-3;  // time constant R1 * C1 (seconds)
const float RD = 1000;     // discharge resistor (ohms)

const float Vin = 5.0;
const float Vout = 5.0;

// LCD I2C address and size
const uint8_t LCD_ADDR = 0x27;
const uint8_t LCD_WIDTH = 16;
const uint8_t LCD_HEIGHT = 2;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT);

ServoDAC dac(PIN_CHARGE, PIN_DISCHARGE, PIN_FEEDBACK, TAU, RD);

unsigned long start_ms;

float f = 0;

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  dac.begin();

  start_ms = millis();

}

void updateLCD(float freq) {
  lcd.clear();
  lcd.print(F("f="));
  lcd.print(freq);
}

float f0 = 0;
float t0 = start_ms * 1.0e-3f; // seconds

void loop() {

  if (Serial.available()) {
    f = Serial.parseFloat();
    Serial.print(F("frequency = "));
    Serial.println(f);
    updateLCD(f);
  }

  float t = millis() * 1.0e-3f; // seconds

  if (0 == f) {
    dac.update(Vout / 2);
  } else {
    float t1 = t0 - (f0 / f) * (t - t0);
    float y = sin(2 * PI * f * (t - t1));

    float v = (y + 1) * (Vout / 2);
    t0 = t1;
    dac.update(v);
  }

  f0 = f;
}
