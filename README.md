# ServoDAC

**ServoDAC** is a digitally controlled, closed-loop DAC implemented using a microcontroller GPIO pin, an RC storage capacitor, and firmware-controlled charge/discharge pulses.

Instead of relying on PWM averaging or resistor ladders, ServoDAC directly **controls the charge on a capacitor** and continuously corrects it using feedback — much like a servo system.

The result is a stable, low-noise analog voltage with predictable dynamics and no PWM carrier ripple.

---

## Concept Overview

Traditional MCU DAC approaches include:

- PWM + RC filtering (carrier ripple, duty-cycle–dependent dynamics)
- R-2R ladders (component tolerance, pin count)
- External DAC ICs (cost, availability, interface complexity)

ServoDAC takes a different approach:

1. Charge a capacitor for a precisely calculated time
2. Discharge it when necessary using a controlled reset path
3. Measure the actual voltage on the capacitor
4. Correct the error every control cycle

This makes the DAC **self-correcting**, insensitive to leakage, and largely independent of component tolerances.

---

## Architecture

### The Example Circuit

Refer to [examples/servodac_test](examples/servodac_test/servodac_test.ino). Here, the test code is responsible for the test input pin, the LCD, and implementing the ServoDAC refresh cycle. 

The ServoDAC library handles the charge, discharge, and feedback pins.

Pin assignments can be changed. The feedback pin must be an analog input pin.

![schematic](schematic.png)

### Core Elements

- **Microcontroller** (Arduino Nano V3)
- **Storage capacitor (C1)** holding the analog voltage
- **Charge resistor (R1)** limiting current from GPIO
- **MOSFET discharge path (Q1 + R_D)** for controlled reset
- **Op-amp buffer (I1A)** isolating C1 from load and ADC
- **Single-pole post-filter (R2/C2)** removing residual spikes
- **Second op-amp buffer (I1B)** driving the final output

The MCU closes the loop by sampling the voltage on C1 and adjusting charge/discharge pulse widths accordingly.

---

## Closed-Loop Operation

Every control cycle (~10 ms):

1. Read **target voltage** (e.g. potentiometer on A3)
2. Read **actual voltage** at C1 (feedback on A2)
3. Compute the error
4. If error exceeds deadband:
   - Apply a **charge pulse** (D3 → R1 → C1), or
   - Apply a **discharge pulse** (D4 → Q1 → R_D)
5. Repeat

Pulse widths are calculated using the **RC exponential equations**, so each correction corresponds directly to the required voltage change.

The system behaves like a servo, not a PWM filter.

---

### Components

- **M1**: Arduino (Nano V3, Uno)
- **I1**: LMC6482 (dual rail-to-rail op-amp, ultra-low input bias)
- **Q1**: 2N7000 N-MOSFET
- **R1**: 1kΩ (charge resistor)  
- **R_D**: 1kΩ (discharge resistor)
- **C1**: 1µF film (Mylar or PP)
- **R2**: 10 kΩ (post-filter)
- **C2**: 100 nF (post-filter)
- **L1**: LCD1602 with I²C backpack

R1, C1 values can be changed, but the library assumes a time constant R1 C1 = 1.0 x 10^{-3}.

To account for any op-amp leakage current, update the target voltage periodically to send a compensating dose of charge to the capacitor.

### Key Connections

- **D3** → R1 → C1 node (charge)
- **D4** → gate resistor → Q1 gate (discharge)
- **A2** → C1 node (feedback)
- **A3** → target input
- **A4 / A5** → LCD1602 (I²C)

## Why This Works Well

- No PWM carrier ripple
- Output defined by **physics + feedback**, not duty cycle
- Leakage and drift are automatically corrected
- Analog performance improves with better op-amp choice
- Filter requirements are minimal (1-pole is sufficient)

## Limitations & Tradeoffs

- Update rate limited by control loop timing
- Step response intentionally smoothed
- Not suitable for high-frequency waveform generation
- Requires careful grounding and op-amp choice

ServoDAC is ideal for **slow, precise analog control**, not audio DACs or waveform synthesis.

---

## Naming Rationale

The name **ServoDAC** reflects the core idea:

> The output voltage is actively servoed to the target value.

This is not PWM filtering — it is closed-loop analog control.
