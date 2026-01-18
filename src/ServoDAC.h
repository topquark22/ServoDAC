#pragma once

#include <Arduino.h>

/**
 * ServoDAC: RC "servo" DAC controller.
 *
 * Drives a charge pin (high pulse then hi-Z) and a discharge pin (active-high pulse)
 * while reading back the output via an ADC feedback pin.
 */
class ServoDAC {
public:
  struct Result {
    float sample_v;          // measured output voltage
    float error_v;           // target_v - sample_v
    unsigned int pulse_us;   // pulse width applied (0 if within deadband)
    bool did_charge;         // true if charge pulse was applied
    bool did_discharge;      // true if discharge pulse was applied
  };

  /**
   * chargePin       Data output pin to charging resistor R1
   * dischargePin    Data output pin to discharge device (e.g. MOSFET gate/base)
   * feedbackPin     Analog input pin sampling the C1 node
   * r1              Charging resistor (ohms)
   * c1              Integrating capacitor (farads)
   * rd              Discharge resistor (ohms)
   */
  ServoDAC(uint8_t chargePin, uint8_t dischargePin, uint8_t feedbackPin,
           float r1, float c1, float rd);

  // --- Tuning (defaults match historical hardcoded values) ---
  // These setters are intended to be called before begin(). By default they are
  // ignored once begin() has been called (safe for control stability).
  ServoDAC& setDeadband(float v);
  ServoDAC& setEpsilon(float v);
  ServoDAC& setMaxChargePulseUs(unsigned long us);
  ServoDAC& setMaxDischargePulseUs(unsigned long us);

  // Configure pins and set safe initial states.
  void begin();

  // Read the feedback pin and convert to volts.
  float readFeedbackVoltage() const;

  // Run one control step toward target_v. Returns the measured sample + applied pulse.
  Result update(float target_v);

  // Utility: convert raw ADC reading to volts using the internal V_IN reference.
  static float adcToVoltage(int raw);

private:
  uint8_t charge_pin_;
  uint8_t discharge_pin_;
  uint8_t feedback_pin_;
  float   r1_;
  float   c1_;
  float   rd_;

  // --- runtime state ---
  bool started_ = false;

  // --- configurable tuning parameters ---
  float deadband_v_ = 0.05f;                 // volts
  float eps_v_ = 0.001f;                     // volts
  unsigned long max_charge_pulse_us_ = 10000UL;
  unsigned long max_discharge_pulse_us_ = 10000UL;

  // Pin-driving primitives.
  void chargePulse(unsigned long pulse_us);
  void dischargePulse(unsigned long pulse_us);

  // Pulse computation (RC model).
  unsigned int calcChargePulse(float target_v, float sample_v);
  unsigned int calcDischargePulse(float target_v, float sample_v);

private:
  // --- constants ---
  static constexpr float EPS_RATIO = 1e-6f;  // dimensionless (log guard)
  static constexpr float V_IN = 5.0f;        // pulse voltage / ADC reference
};
