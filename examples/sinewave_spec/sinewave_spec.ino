#include <math.h>

#include <LiquidCrystal_I2C.h>

#include "ServoDAC.h"

// --- wiring pins (matching the original sketch defaults) ---
const uint8_t PIN_CHARGE = 3;     // charge pin (pulse high, then hi-Z)
const uint8_t PIN_DISCHARGE = 4;  // discharge pin (active-high)
const uint8_t PIN_FEEDBACK = A2;  // feedback pin

const float R1 = 2.2e3;  // Charging resistor (ohms)
const float C1 = 470e-9; // Integrating capacitor (farads)
const float R_D = 2.2e3;  // discharge resistor (ohms)

const float Vin = 5.0;
const float Vout = 5.0;

// LCD I2C address and size
const uint8_t LCD_ADDR = 0x27;
const uint8_t LCD_WIDTH = 16;
const uint8_t LCD_HEIGHT = 2;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT);

// Control loop
static const unsigned long UPDATE_INTERVAL_MS = 2;

// Update LCD no faster than this
static const unsigned long LCD_RATE_MS = 250;
static const unsigned long LCD_RATE_FRAMES = LCD_RATE_MS / UPDATE_INTERVAL_MS;

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

void updateLCD(float freq) {
  lcd.clear();
  lcd.print(F("f="));
  lcd.print(freq);
}

void loop() {
  static unsigned long next_us = micros();

  static float f = 0.0f;

  if (Serial.available()) {
    f = Serial.parseFloat();
    while (Serial.available()) { Serial.read(); }
    Serial.print(F("frequency = "));
    Serial.println(f);
    updateLCD(f);
  }

  float y = 0.0f;
  
  if (f > 0.0f) {
    const unsigned long elapsed_ms = millis() - start_ms;

    // Keep the floating-point value small by wrapping time to one cycle.
    const float period_ms = 1000.0f / f;
    const float cycle_ms = fmodf((float)elapsed_ms, period_ms);

    const float phase = 2.0f * PI * (cycle_ms / period_ms);
    y = sinf(phase);
  }

  const float g_target_v = (y + 1.0f) * (Vout * 0.5f);

  // --- control step ---
  const ServoDAC::Result r = dac.update(g_target_v);
  
  // --- wait until next frame boundary ---
  next_us += (unsigned long)UPDATE_INTERVAL_MS * 1000UL;
  while ((long)(micros() - next_us) < 0)
  {
    // spin
  }

}
