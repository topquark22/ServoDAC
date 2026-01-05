#include "dejitter.h"

Dejitter::Dejitter(uint8_t pin)
: pin_(pin) {
    pinMode(pin, INPUT);
    prev_value = -2;
}

int Dejitter::read() {
  int value = analogRead(pin_);
  if (abs(value - prev_value) < 2) {
    return prev_value;
  }
  prev_value = value;
  return value;
}