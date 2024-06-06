#include "ControlGainHandler.h"

// Implement the member functions of ControlGainHandler class here

// Constructor
ControlGainHandler::ControlGainHandler() {
  kd = 0;
  ki = 0;
  kp = 0;
  lastError = 0;
  integral = 0;
}

ControlGainHandler::ControlGainHandler(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  lastError = 0;
  integral = 0;
}

// Destructor
ControlGainHandler::~ControlGainHandler() {
  // Clean up any dynamically allocated resources here
}

// Reset the error and integral terms
void ControlGainHandler::reset() {
  lastError = 0;
  integral = 0;
}

float ControlGainHandler::calculatePID(float error) {
  // Update the integral term
  integral += error;

  // Calculate the proportional term
  float proportional = kp * error;

  // Calculate the integral term
  float integral = ki * this->integral;

  // Calculate the derivative term
  float derivative = kd * (error - lastError);

  // Update the last error
  lastError = error;

  // Return the PID output
  return proportional + integral + derivative;
}