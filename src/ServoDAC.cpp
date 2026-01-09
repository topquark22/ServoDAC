#include "ServoDAC.h"

#include <math.h>   // logf, fabsf

ServoDAC::ServoDAC(uint8_t chargePin, uint8_t dischargePin, uint8_t feedbackPin, float tau, float rd)
: charge_pin_(chargePin),
  discharge_pin_(dischargePin),
  feedback_pin_(feedbackPin),
  tau_(tau),
  rd_(rd) {}

void ServoDAC::begin() {
  // Discharge pin is a digital output (assumed active-high discharge).
  pinMode(discharge_pin_, OUTPUT);
  digitalWrite(discharge_pin_, LOW);

  // Feedback pin is analog input.
  pinMode(feedback_pin_, INPUT);

  // Charge pin should default to hi-Z (input) and LOW.
  pinMode(charge_pin_, INPUT);
  digitalWrite(charge_pin_, LOW);
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
  pinMode(charge_pin_, INPUT); // high-impedance
  digitalWrite(charge_pin_, LOW);
}

void ServoDAC::dischargePulse(unsigned long pulse_us) {
  digitalWrite(discharge_pin_, HIGH);
  delayMicroseconds(pulse_us);
  digitalWrite(discharge_pin_, LOW);
}

unsigned int ServoDAC::calcChargePulse(float target, float sample) {
  if (target <= sample) return 0;

  // Clamp target/sample to sane range
  if (target > V_IN - EPS_V) target = V_IN - EPS_V;
  if (sample > V_IN - EPS_V) sample = V_IN - EPS_V;

  // If sample is already basically at the rail, you can't charge higher meaningfully
  if ((V_IN - sample) <= EPS_V) return 0;

  const float denom = (V_IN - sample);
  const float numer = (V_IN - target);

  float ratio = numer / denom;

  // ratio should be in (0, 1). If it's <=0, we'd need infinite time -> saturate.
  if (ratio <= EPS_RATIO) {
    return (unsigned int)MAX_CHARGE_PULSE_US;
  }
  if (ratio >= 1.0f) return 0; // target ~= sample

  const float t_sec = -tau_ * logf(ratio);
  const float t_us_f = t_sec * 1e6f;

  if (t_us_f <= 0.0f) return 0;
  if (t_us_f > (float)MAX_CHARGE_PULSE_US) return (unsigned int)MAX_CHARGE_PULSE_US;

  return (unsigned int)(t_us_f + 0.5f); // round
}

unsigned int ServoDAC::calcDischargePulse(float target, float sample) {
  if (target >= sample) return 0;

  // Clamp away from 0 to avoid ratio=0
  if (target < EPS_V) target = EPS_V;
  if (sample < EPS_V) sample = EPS_V;


  const float ratio = target / sample;

  // ratio should be in (0, 1). If <=0, infinite time -> saturate.
  if (ratio <= EPS_RATIO) return (unsigned int)MAX_DISCHARGE_PULSE_US;
  if (ratio >= 1.0f) return 0;

  const float t_sec = -tau_ * logf(ratio);
  const float t_us_f = t_sec * 1e6f;

  if (t_us_f <= 0.0f) return 0;
  if (t_us_f > (float)MAX_DISCHARGE_PULSE_US) return (unsigned int)MAX_DISCHARGE_PULSE_US;

  return (unsigned int)(t_us_f + 0.5f); // round
}

ServoDAC::Result ServoDAC::update(float target_v) {
  Result r{};
  r.sample_v = readFeedbackVoltage();
  r.error_v = target_v - r.sample_v;
  r.pulse_us = 0;
  r.did_charge = false;
  r.did_discharge = false;

  if (fabsf(r.error_v) >= DEADBAND_V) {
    if (r.error_v > 0.0f) {
      r.pulse_us = calcChargePulse(target_v, r.sample_v);
      if (r.pulse_us > MAX_CHARGE_PULSE_US) r.pulse_us = (unsigned int)MAX_CHARGE_PULSE_US;
      if (r.pulse_us > 0) {
        chargePulse(r.pulse_us);
        r.did_charge = true;
      }
    } else {
      r.pulse_us = calcDischargePulse(target_v, r.sample_v);
      if (r.pulse_us > MAX_DISCHARGE_PULSE_US) r.pulse_us = (unsigned int)MAX_DISCHARGE_PULSE_US;
      if (r.pulse_us > 0) {
        dischargePulse(r.pulse_us);
        r.did_discharge = true;
      }
    }
  }

  return r;
}
