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
