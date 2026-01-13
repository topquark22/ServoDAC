#include <math.h>

#include <LiquidCrystal_I2C.h>

#include "ServoDAC.h"

// --- wiring pins (matching the original sketch defaults) ---
const uint8_t PIN_CHARGE = 5;     // charge pin (pulse high, then hi-Z)
const uint8_t PIN_DISCHARGE = 4;  // discharge pin (active-high)
const uint8_t PIN_FEEDBACK = A2;  // feedback pin

// R1 = 2.2K
// C1 = 470nf
// Rd = 1.0k
const float R1 = 2.2e3;  // Charging resistor (ohms)
const float C1 = 470e-9; // Integrating capacitor (farads)
const float R_D = 1.0e3;  // discharge resistor (ohms)

const float Vin = 5.0;
const float Vout = 5.0;

// LCD I2C address and size
const uint8_t LCD_ADDR = 0x27;
const uint8_t LCD_WIDTH = 16;
const uint8_t LCD_HEIGHT = 2;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT);

ServoDAC dac(PIN_CHARGE, PIN_DISCHARGE, PIN_FEEDBACK, R1, C1, R_D);

unsigned long start_ms;

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  start_ms = millis();
  dac.begin();

}

void updateLCD(float freq) {
  lcd.clear();
  lcd.print(F("f="));
  lcd.print(freq);
}

float f = 0;

void loop() {

  if (Serial.available()) {
    f = Serial.parseFloat();
    while (Serial.available()) { Serial.read(); }
    Serial.print(F("frequency = "));
    Serial.println(f);
    updateLCD(f);
  }

  float t = (millis() - start_ms) * 1.0e-3f; // seconds
  float y = sin(2 * PI * f * t);
  float v = (y + 1) * (Vout / 2);
  dac.update(v);
}
