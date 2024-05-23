#include "MovingAverageSensor.h"
#include <Arduino.h>

MovingAverageSensor::MovingAverageSensor(int pin) : pin(pin) {}

void MovingAverageSensor::read() {
  values[index] = analogRead(pin);
  index = (index + 1) % BUFFER_SIZE;
}

int MovingAverageSensor::average() const {
  int sum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += values[i];
  }
  return sum / BUFFER_SIZE;
}