#include "SimpleSensor.h"
#include <Arduino.h>

SimpleSensor::SimpleSensor(int pin) : pin(pin) {}

void SimpleSensor::read() {
  lastReading = analogRead(pin);
}

int SimpleSensor::cleanRead() {
  lastReading = analogRead(pin);
  return lastReading;
}

int SimpleSensor::getLastReading() {
  return lastReading;
}
