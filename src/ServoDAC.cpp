#include "ServoDAC.h"

#include <math.h>  // logf, fabsf

ServoDAC::ServoDAC(uint8_t chargePin, uint8_t dischargePin, uint8_t feedbackPin,
                   float r1, float c1, float rd)
    : charge_pin_(chargePin),
      discharge_pin_(dischargePin),
      feedback_pin_(feedbackPin),
      r1_(r1),
      c1_(c1),
      rd_(rd) {
}

// --- Tuning setters (intended to be called before begin()) ---
ServoDAC& ServoDAC::setDeadband(float v) {
  if (!started_) {
    deadband_v_ = (v < 0.0f) ? 0.0f : v;
  }
  return *this;
}

ServoDAC& ServoDAC::setEpsilon(float v) {
  if (!started_) {
    // Avoid non-positive epsilon (used in clamps)
    eps_v_ = (v <= 0.0f) ? 1.0e-6f : v;
  }
  return *this;
}

ServoDAC& ServoDAC::setMaxChargePulseUs(unsigned long us) {
  if (!started_) {
    max_charge_pulse_us_ = (us == 0UL) ? 1UL : us;
  }
  return *this;
}

ServoDAC& ServoDAC::setMaxDischargePulseUs(unsigned long us) {
  if (!started_) {
    max_discharge_pulse_us_ = (us == 0UL) ? 1UL : us;
  }
  return *this;
}

void ServoDAC::begin() {
  // Discharge pin is a digital output (assumed active-high discharge).
  pinMode(discharge_pin_, OUTPUT);
  digitalWrite(discharge_pin_, LOW);

  // Feedback pin is analog input.
  pinMode(feedback_pin_, INPUT);

  // Charge pin defaults to hi-Z (input) and LOW.
  pinMode(charge_pin_, INPUT);
  digitalWrite(charge_pin_, LOW);

  started_ = true;
}

float ServoDAC::adcToVoltage(int raw) {
  // Arduino ADC is 10-bit: 0..1023 represents 0..V_IN
  return raw * V_IN / 1023.0f;
}

float ServoDAC::readFeedbackVoltage() const {
  return adcToVoltage(analogRead(feedback_pin_));
}

void ServoDAC::chargePulse(unsigned long pulse_us) {
  pinMode(charge_pin_, OUTPUT);
  digitalWrite(charge_pin_, HIGH);
  delayMicroseconds(pulse_us);
  pinMode(charge_pin_, INPUT);  // high-impedance
  digitalWrite(charge_pin_, LOW);
}

void ServoDAC::dischargePulse(unsigned long pulse_us) {
  digitalWrite(discharge_pin_, HIGH);
  delayMicroseconds(pulse_us);
  digitalWrite(discharge_pin_, LOW);
}

unsigned int ServoDAC::calcChargePulse(float target, float sample) {
  if (target <= sample) return 0;

  const float eps = eps_v_;

  // Clamp target/sample to sane range
  if (target > V_IN - eps) target = V_IN - eps;
  if (sample > V_IN - eps) sample = V_IN - eps;

  // If sample is already basically at the rail, you can't charge higher meaningfully
  if ((V_IN - sample) <= eps) return 0;

  const float denom = (V_IN - sample);
  const float numer = (V_IN - target);

  const float ratio = numer / denom;

  // ratio should be in (0, 1). If it's <= 0, we'd need infinite time -> saturate.
  if (ratio <= EPS_RATIO) {
    return (unsigned int)max_charge_pulse_us_;
  }
  if (ratio >= 1.0f) return 0;  // target ~= sample

  const float t_sec = -r1_ * c1_ * logf(ratio);
  const float t_us_f = t_sec * 1e6f;

  if (t_us_f <= 0.0f) return 0;
  if (t_us_f > (float)max_charge_pulse_us_) return (unsigned int)max_charge_pulse_us_;

  return (unsigned int)(t_us_f + 0.5f);  // round
}

unsigned int ServoDAC::calcDischargePulse(float target, float sample) {
  if (target >= sample) return 0;

  const float eps = eps_v_;

  // Clamp away from 0 to avoid ratio=0
  if (target < eps) target = eps;
  if (sample < eps) sample = eps;

  const float ratio = target / sample;

  // ratio should be in (0, 1). If it's <= 0, we'd need infinite time -> saturate.
  if (ratio <= EPS_RATIO) return (unsigned int)max_discharge_pulse_us_;
  if (ratio >= 1.0f) return 0;

  const float t_sec = -rd_ * c1_ * logf(ratio);
  const float t_us_f = t_sec * 1e6f;

  if (t_us_f <= 0.0f) return 0;
  if (t_us_f > (float)max_discharge_pulse_us_) return (unsigned int)max_discharge_pulse_us_;

  return (unsigned int)(t_us_f + 0.5f);  // round
}

ServoDAC::Result ServoDAC::update(float target_v) {
  Result r{};
  r.sample_v = readFeedbackVoltage();
  r.error_v = target_v - r.sample_v;
  r.pulse_us = 0;
  r.did_charge = false;
  r.did_discharge = false;

  if (fabsf(r.error_v) >= deadband_v_) {
    if (r.error_v > 0.0f) {
      r.pulse_us = calcChargePulse(target_v, r.sample_v);
      if (r.pulse_us > max_charge_pulse_us_) r.pulse_us = (unsigned int)max_charge_pulse_us_;
      if (r.pulse_us > 0U) {
        chargePulse(r.pulse_us);
        r.did_charge = true;
      }
    } else {
      r.pulse_us = calcDischargePulse(target_v, r.sample_v);
      if (r.pulse_us > max_discharge_pulse_us_) r.pulse_us = (unsigned int)max_discharge_pulse_us_;
      if (r.pulse_us > 0U) {
        dischargePulse(r.pulse_us);
        r.did_discharge = true;
      }
    }
  }

  return r;
}
