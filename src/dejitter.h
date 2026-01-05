#pragma once

#include <Arduino.h>

/**
 * Dejitter an analog input pin
 */
class Dejitter {
public:
  Dejitter(uint8_t pin);

  int read();  // TODO fix return type to that of analogRead()

private:
  uint8_t pin_;
  int prev_value;
};