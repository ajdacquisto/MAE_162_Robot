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

// Reset the error term
void ControlGainHandler::resetError() { lastError = 0; }

// Reset the integral term
void ControlGainHandler::resetIntegral() {
  lastIntegralResetTime = millis();
  integral = 0;
}

// Reset the error and integral terms
void ControlGainHandler::reset() {
  lastError = 0;
  integral = 0;
}

// Set the last error
void ControlGainHandler::setLastError(int error) { lastError = error; }

// Get the last error
int ControlGainHandler::getLastError() { return lastError; }

// Get the integral term
int ControlGainHandler::getIntegral() { return integral; }

// Set the proportional gain
void ControlGainHandler::setKp(float kp) { this->kp = kp; }

// Set the integral gain
void ControlGainHandler::setKi(float ki) { this->ki = ki; }

// Set the derivative gain
void ControlGainHandler::setKd(float kd) { this->kd = kd; }

// Get the proportional gain
float ControlGainHandler::getKp() { return kp; }

// Get the integral gain
float ControlGainHandler::getKi() { return ki; }

// Get the derivative gain
float ControlGainHandler::getKd() { return kd; }

void ControlGainHandler::incrementIntegral(int error) {
  // Increment the integral term
  integral += error;
}

unsigned long ControlGainHandler::getLastIntegralResetTime() {
  return lastIntegralResetTime;
}