#include <LiquidCrystal_I2C.h>

#include "servodac.h"

// R1 = 2.2K
// C1 = 470nF
// tau = R1 * C1 = 1.0 ms
const float TAU = 1.0e-3; // time constant R1 * C1 (seconds)
const float RD = 1000;    // discharge resistor (ohms)

// --- wiring pins (matching the original sketch defaults) ---
const uint8_t PIN_CHARGE     = 5;   // charge pin (pulse high, then hi-Z)
const uint8_t PIN_DISCHARGE  = 4;   // discharge pin (active-high)
const uint8_t PIN_FEEDBACK   = A2;  // feedback pin

const uint8_t PIN_TEST_IN    = A3;  // target pin (ADC)

// --- loop timing / UI ---
const unsigned int UPDATE_INTERVAL_MS = 10; // control loop period
const unsigned int LCD_RATE_MS = 250;       // update LCD every N ms
const unsigned int LCD_RATE_FRAMES = LCD_RATE_MS / UPDATE_INTERVAL_MS;

// LCD I2C addr and size
const uint8_t LCD_ADDR = 0x27;
const uint8_t LCD_WIDTH = 16;
const uint8_t LCD_HEIGHT = 2;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT);

// ServoDAC instance (chargePin, dischargePin, feedbackPin)
ServoDAC dac(PIN_CHARGE, PIN_DISCHARGE, PIN_FEEDBACK, TAU, RD);

static void updateLCD(float target, const ServoDAC::Result& r) {
  // target voltage
  lcd.setCursor(0, 0);
  lcd.print(target);
  lcd.print(F("V  "));

  // pulse width
  lcd.setCursor(0, 1);
  lcd.print(r.pulse_us);
  lcd.print(F("us   "));

  // actual output voltage
  lcd.setCursor(8, 0);
  lcd.print(r.sample_v);
  lcd.print(F("V  "));


  // error
  lcd.setCursor(7, 1);
  lcd.print(r.error_v);
  lcd.print(F("V  "));
}

unsigned int loopCt = 0;

void setup() {
  // target input pin
  pinMode(PIN_TEST_IN, INPUT);

  // servo-dac pins
  dac.begin();

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void loop() {
  static unsigned long next_us = 0;
  if (next_us == 0) next_us = micros();

  // --- target sample ---
  const int target_raw = analogRead(PIN_TEST_IN);
  const float target_v = ServoDAC::adcToVoltage(target_raw);

  // --- control step ---
  const ServoDAC::Result r = dac.update(target_v);

  if (loopCt == 0) {
    updateLCD(target_v, r);
  }
  loopCt = (loopCt + 1) % LCD_RATE_FRAMES;

  // --- wait until next frame boundary ---
  next_us += (unsigned long)UPDATE_INTERVAL_MS * 1000UL;
  while ((long)(micros() - next_us) < 0) {
    // do nothing
  }
}
