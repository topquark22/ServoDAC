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

/*
 * chargePin       Data output pin to charging resistor R1
 * dischargePin    Data output pin to Q1 MOSFET base (via protection resistor)
 * feedbackPin     Analog input pin to C1 node
 * tau             RC time constant R1 * R2 (seconds)
 * rd              Discharge resistor (ohms)
 */
  ServoDAC(uint8_t chargePin, uint8_t dischargePin, uint8_t feedbackPin, float tau, float rd);

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
  float   tau_;
  float   rd_;

  // Pin-driving primitives.
  void chargePulse(unsigned long pulse_us);
  void dischargePulse(unsigned long pulse_us);

  // Pulse computation (RC model).
  unsigned int calcChargePulse(float target_v, float sample_v);
  unsigned int calcDischargePulse(float target_v, float sample_v);

private:
  // --- constants specific to the servo-DAC control ---
  static constexpr float EPS_V = 0.001f;     // 1 mV
  static constexpr float EPS_RATIO = 1e-6f;  // dimensionless
  static constexpr float V_IN = 5.0f;        // pulse voltage / ADC reference

  static constexpr float DEADBAND_V = 0.02f; // volts

  static constexpr unsigned long MAX_CHARGE_PULSE_US = 12000UL;
  static constexpr unsigned long MAX_DISCHARGE_PULSE_US = 12000UL;
};
