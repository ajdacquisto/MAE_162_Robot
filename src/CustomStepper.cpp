#include "CustomStepper.h"

// Constructor to initialize the stepper motor pins and steps per revolution
CustomStepper::CustomStepper(int dirPin, int stepPin, int enablePin, int stepsPerRev) {
  this->dirPin = dirPin;
  this->stepPin = stepPin;
  this->enablePin = enablePin;
  this->stepsPerRev = stepsPerRev;
  
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  // Enable the driver
  digitalWrite(enablePin, LOW);
}

// Method to rotate the motor clockwise
void CustomStepper::rotateClockwise(int numSteps, int rpm) {
  int delayMicroseconds = calculateDelayMicroseconds(rpm);
  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < numSteps; i++) {
    stepMotor(delayMicroseconds);
  }
}

// Method to rotate the motor counterclockwise
void CustomStepper::rotateCounterclockwise(int numSteps, int rpm) {
  int delayMicroseconds = calculateDelayMicroseconds(rpm);
  digitalWrite(dirPin, LOW);
  for (int i = 0; i < numSteps; i++) {
    stepMotor(delayMicroseconds);
  }
}

// Method to calculate the delay in microseconds based on RPM
int CustomStepper::calculateDelayMicroseconds(int rpm) {
  int stepRate = (rpm * stepsPerRev) / 60; // Steps per second
  return 1000000 / stepRate; // Microseconds per step
}

// Method to step the motor
void CustomStepper::stepMotor(int delayMicroseconds_) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delayMicroseconds_);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayMicroseconds_);
}
