#include <Arduino.h>

#include <LiquidCrystal_I2C.h>

#include "servodac.h"

/*
  ServoDAC - "spec" example (Serial programmable output)

  Type a voltage into the Serial Monitor (e.g. 1.25) and press Enter.
  The sketch will servo the RC DAC output to that voltage.

  Notes:
  - Valid range is 0.0 to 5.0 (Arduino Nano default ADC reference).
  - Serial Monitor line ending: Newline (or Both NL & CR) is fine.
  - Uses Serial.parseFloat() (works on AVR; strtof() does not).
*/

// --- wiring pins (adjust if your hardware differs) ---
static const uint8_t PIN_CHARGE = 5;    // charge pin (pulse high, then hi-Z)
static const uint8_t PIN_DISCHARGE = 4; // discharge pin (active-high)
static const uint8_t PIN_FEEDBACK = A2; // feedback pin (ADC)

// RC constants (match your build; these mirror the other examples)
// tau = R * C  (seconds)
static const float TAU = 1.0e-3f; // ~1ms (example value)
static const float RD = 1000.0f;  // discharge resistor (ohms)

// Control loop
static const unsigned int UPDATE_INTERVAL_MS = 10;

// LCD (16x2 I2C; adjust address if needed)
static const uint8_t LCD_ADDR = 0x27;
static const uint8_t LCD_WIDTH = 16;
static const uint8_t LCD_HEIGHT = 2;

// Update LCD no faster than this
static const unsigned int LCD_RATE_MS = 250;
static const unsigned int LCD_RATE_FRAMES = LCD_RATE_MS / UPDATE_INTERVAL_MS;

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_WIDTH, LCD_HEIGHT);
ServoDAC dac(PIN_CHARGE, PIN_DISCHARGE, PIN_FEEDBACK, TAU, RD);

static float g_target_v = 0.0f;

static void printHelp()
{
  Serial.println();
  Serial.println(F("ServoDAC spec example"));
  Serial.println(F("Enter a voltage (0.0 to 5.0) and press Enter."));
  Serial.println(F("Examples: 0, 1.25, 2.500, 5"));
  Serial.println(F("Command:  ?  (prints this help)"));
  Serial.println();
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

static bool tryHandleCommand()
{
  // Non-blocking: only act if something is available
  if (Serial.available() <= 0)
    return false;

  // Peek for '?' as a "help" command
  int c = Serial.peek();
  if (c == '?')
  {
    Serial.read(); // consume '?'
    // consume remainder of line (if any)
    while (Serial.available() > 0)
    {
      char ch = (char)Serial.read();
      if (ch == '\n' || ch == '\r')
        break;
    }
    printHelp();
    return true;
  }

  // Otherwise, parse a float (AVR-safe)
  float v = Serial.parseFloat(); // reads until it finds a number; stops at non-number

  // Flush remainder of the line so the next read is clean
  while (Serial.available() > 0)
  {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r')
      break;
  }

  // If parseFloat found nothing, it returns 0.0 after timeout.
  // We keep it simple: treat it as a valid "0" command unless input was junk.
  // If you want stricter behavior, shorten Serial timeout and/or add a prompt.
  if (v < 0.0f)
    v = 0.0f;
  if (v > 5.0f)
    v = 5.0f;

  g_target_v = v;

  Serial.print(F("Target set to "));
  Serial.print(g_target_v, 3);
  Serial.println(F(" V"));

  return true;
}

void setup()
{
  Serial.begin(115200);
  // On Nano this returns immediately; on native-USB boards it may wait.
  while (!Serial)
  { /* no-op */
  }

  // Make parseFloat snappier (default is 1000 ms)
  Serial.setTimeout(50);

  dac.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  printHelp();
  Serial.print(F("Initial target: "));
  Serial.print(g_target_v, 3);
  Serial.println(F(" V"));
}

void loop()
{
  static unsigned long next_us = micros();
  static unsigned int loopCt = 0;

  // --- serial input (optional each frame) ---
  (void)tryHandleCommand();

  // --- control step ---
  const ServoDAC::Result r = dac.update(g_target_v);

  if (loopCt == 0)
  {
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
