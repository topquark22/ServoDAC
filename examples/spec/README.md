# Spec (Serial-Programmable Output)

This example sets the ServoDAC output voltage from the Serial Monitor.

Type a voltage (in volts) and press Enter, for example:

- `0`
- `1.25`
- `2.500`
- `5`

The value is clamped to the valid range `0.0 … 5.0`.

## Hardware

Same wiring and RC network as the other examples:

- `PIN_CHARGE = D5`
- `PIN_DISCHARGE = D4`
- `PIN_FEEDBACK = A2`

The sketch also uses a 16×2 I2C LCD (default address `0x27`).

## Software requirements

- ServoDAC (this library)
- LiquidCrystal_I2C (installed via Arduino Library Manager)

## Notes

- Serial speed is **115200** baud.
- In the Arduino Serial Monitor, set line ending to **Newline** (or **Both NL & CR**).
