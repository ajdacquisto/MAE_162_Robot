#include "NewIRSensor.h"

// Implement the member functions of the NewIRSensor class here

// Constructor
/**
 * @brief Constructs a new instance of the NewIRSensor class.
 *
 * @param pin1 The pin number for sensor 1.
 * @param pin2 The pin number for sensor 2.
 * @param pin3 The pin number for sensor 3.
 * @param pin4 The pin number for sensor 4.
 * @param pin5 The pin number for sensor 5.
 */
NewIRSensor::NewIRSensor(int pin1, int pin2, int pin3, int pin4, int pin5) {
  pin1_ = pin1;
  pin2_ = pin2;
  pin3_ = pin3;
  pin4_ = pin4;
  pin5_ = pin5;
  // Initialize any other necessary variables or perform setup here
}

// Destructor
/**
 * @brief Destructor for the NewIRSensor class.
 *
 * This destructor does not perform any specific actions.
 */
NewIRSensor::~NewIRSensor() {
  // No action needed for destructor
}

// Initialize function
/**
 * Initializes the NewIRSensor object by setting the pin modes for the sensor
 * pins. This function should be called before using any other functions of the
 * NewIRSensor class.
 */
void NewIRSensor::init() {
  pinMode(pin1_, INPUT);
  pinMode(pin2_, INPUT);
  pinMode(pin3_, INPUT);
  pinMode(pin4_, INPUT);
  pinMode(pin5_, INPUT);
}

/**
 * Reads the sensor data from the digital sensor pins and returns the result as
 * a uint8_t value.
 *
 * @return The sensor data as a uint8_t value.
 */
uint8_t NewIRSensor::readSensorData() {
  uint8_t sensorData = 0;
  // Code to read sensor data goes here
  // Assuming pin1, pin2, pin3, pin4, pin5 are the digital sensor pins
  sensorData |= !digitalRead(pin1_) << 0;
  sensorData |= !digitalRead(pin2_) << 1;
  sensorData |= !digitalRead(pin3_) << 2;
  sensorData |= !digitalRead(pin4_) << 3;
  sensorData |= !digitalRead(pin5_) << 4;
  return sensorData;
}
