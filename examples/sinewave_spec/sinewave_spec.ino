#include <math.h>

#include <LiquidCrystal_I2C.h>

#include "ServoDAC.h"

// --- wiring pins (matching the original sketch defaults) ---
const uint8_t PIN_CHARGE = 3;     // charge pin (pulse high, then hi-Z)
const uint8_t PIN_DISCHARGE = 4;  // discharge pin (active-high)
const uint8_t PIN_FEEDBACK = A2;  // feedback pin

// R1 = 22K
// C1 = 47nf
// Rd = 22k
const float R1 = 2.2e3;  // Charging resistor (ohms)
const float C1 = 470e-9; // Integrating capacitor (farads)
const float R_D = 2.2e3;  // discharge resistor (ohms)

const float Vin = 5.0;
const float Vout = 5.0;

// Control loop
static const unsigned long UPDATE_INTERVAL_MS = 2;

// Update LCD no faster than this
static const unsigned long LCD_RATE_MS = 250;
static const unsigned long LCD_RATE_FRAMES = LCD_RATE_MS / UPDATE_INTERVAL_MS;

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

  // Explicitly apply ServoDAC default tuning values (overrideable).
  // These correspond to the historical hardcoded defaults.
  Serial.begin(115200);

  dac.setDeadband(0.02f);
  dac.setEpsilon(0.001f);
  dac.setMaxChargePulseUs(12000UL);
  dac.setMaxDischargePulseUs(12000UL);

  dac.begin();
}

static void updateLCD(float target, const ServoDAC::Result &r)
{
  // Row 0: target and measured
  lcd.setCursor(0, 0);
  lcd.print(target, 3);
  lcd.print(F("V "));

  lcd.setCursor(8, 0);
  lcd.print(r.sample_v, 3);
  lcd.print(F("V "));

  // Row 1: pulse width and error
  lcd.setCursor(0, 1);
  lcd.print(r.pulse_us);
  lcd.print(F("us   "));

  lcd.setCursor(8, 1);
  lcd.print(r.error_v, 3);
  lcd.print(F("V "));
}

void loop() {
  static unsigned long next_us = micros();
  static unsigned int loopCt = 0;

  static float f = 0.0f;

  if (Serial.available()) {
    f = Serial.parseFloat();
    while (Serial.available()) { Serial.read(); }
    Serial.print(F("frequency = "));
    Serial.println(f);
  }

  float t = (millis() - start_ms) * 1.0e-3f; // seconds
  float y = sin(2 * PI * f * t);
  float g_target_v = (y + 1) * (Vout / 2);

  // --- control step ---
  const ServoDAC::Result r = dac.update(g_target_v);
  if (loopCt == 0) {
    updateLCD(g_target_v, r);
  }
  loopCt = (loopCt + 1) % LCD_RATE_FRAMES;

  // --- wait until next frame boundary ---
  next_us += (unsigned long)UPDATE_INTERVAL_MS * 1000UL;
  while ((long)(micros() - next_us) < 0)
  {
    // spin
  }

}
