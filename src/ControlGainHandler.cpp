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

ControlGainHandler::ControlGainHandler(float kp, float kd, float ki) {
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
    lastError = 0;
    integral = 0;
}

// Destructor
ControlGainHandler::~ControlGainHandler() {
    // Clean up any dynamically allocated resources here
}

void ControlGainHandler::resetError() {
    // Reset the error to zero
    lastError = 0;
}

void ControlGainHandler::resetIntegral() {
    // Reset the integral to zero
    integral = 0;
}

void ControlGainHandler::reset() {
    // Reset both the error and integral to zero
    //resetError();
    lastError = 0;
    integral = 0;
}

void ControlGainHandler::setLastError(int error) {
    // Set the last error value
    lastError = error;
}

void ControlGainHandler::setIntegral(int integral) {
    // Set the integral value
    this->integral = integral;
}

int ControlGainHandler::getLastError() {
    // Return the last error value
    return lastError;
}

int ControlGainHandler::getIntegral() {
    // Return the integral value
    return integral;
}

void ControlGainHandler::setKp(float kp) {
    // Set the proportional gain
    this->kp = kp;
}

void ControlGainHandler::setKd(float kd) {
    // Set the derivative gain
    this->kd = kd;
}

void ControlGainHandler::setKi(float ki) {
    // Set the integral gain
    this->ki = ki;
}

float ControlGainHandler::getKp() {
    // Return the proportional gain
    return kp;
}

float ControlGainHandler::getKd() {
    // Return the derivative gain
    return kd;
}

float ControlGainHandler::getKi() {
    // Return the integral gain
    return ki;
}

void ControlGainHandler::incrementIntegral(int error) {
    // Increment the integral term
    integral += error;
}