#include "SensorController.h"

// Constructor
SensorController::SensorController()
    : encoderA(ENCODER_PIN_A1, ENCODER_PIN_A2),     // Encoder A
      encoderB(ENCODER_PIN_B1, ENCODER_PIN_B2),     // Encoder B
      hc(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN), // Ultrasonic sensor
      lineSensorA1(LINE_SENSOR_PIN_A1),             // Line sensor A1
      lineSensorA2(LINE_SENSOR_PIN_A2),             // Line sensor A2
      lineSensorA3(LINE_SENSOR_PIN_A3),             // Line sensor A3
      lineSensorB1(LINE_SENSOR_PIN_B1),             // Line sensor B1
      lineSensorB2(LINE_SENSOR_PIN_B2),             // Line sensor B2
      lineSensorB3(LINE_SENSOR_PIN_B3)              // Line sensor B3
{
  ultrasonicMemory = 0;
  lineSensorAThreshold = LINE_SENSOR_A_THRESHOLD;
}

// Destructor
SensorController::~SensorController() {
  // Clean up any resources
}

// ===== SETUP FUNCTIONS =====
void SensorController::zeroEncoders() {
  encoderA.write(0);
  encoderB.write(0);
}

// ===== SENSOR READINGS =====
long SensorController::readEncoderA() { return encoderA.read(); }

long SensorController::readEncoderB() { return encoderB.read(); }

void SensorController::readLineSensorA() {
  lineSensorA1.read();
  lineSensorA2.read();
  lineSensorA3.read();
}

void SensorController::readLineSensorB() {
  lineSensorB1.read();
  lineSensorB2.read();
  lineSensorB3.read();
}

int SensorController::combineLineResult(int avg1, int avg2, int avg3) {

  // CONVENTION: 1 = black ON-TARGET, 0 = white OFF-TARGET
  int lineSensorValueA1 = (avg1 > getLineSensorAThreshold()) ? 1 : 0;
  int lineSensorValueA2 = (avg2 > getLineSensorAThreshold()) ? 1 : 0;
  int lineSensorValueA3 = (avg3 > getLineSensorAThreshold()) ? 1 : 0;

  // COMBINE values into one variable (e.g. 001, 000, 111, 101, etc)
  int lineSensorValue =
      (lineSensorValueA1 << 2) | (lineSensorValueA2 << 1) | lineSensorValueA3;

  return lineSensorValue;
}

int SensorController::determineError(int lineSensorValue) {
  // Determine the error based on the line sensor value
  int error = 0;

  switch (lineSensorValue) {
  case 0b000:
    // Robot is off the line, keep last known direction
    error = 99;
    break;
  case 0b001:
    // Robot needs to turn hard right
    error = +2;
    break;
  case 0b011:
    // Robot needs to turn slightly right
    error = +1;
    break;
  case 0b010:
    // Robot is centered
    error = 0;
    break;
  case 0b110:
    // Robot needs to turn slightly left
    error = -1;
    break;
  case 0b100:
    // Robot needs to hard left
    error = -2;
    break;
  case 0b101:
    // Robot is centered
    error = 0;
    break;
  default:
    // Robot is centered
    error = 0;
    break;
  }

  return error;
}

int SensorController::getLineResultA() {
  return combineLineResult(lineSensorA1.average(), lineSensorA2.average(),
                           lineSensorA3.average());
}

int SensorController::getLineResultB() {
  return combineLineResult(lineSensorB1.average(), lineSensorB2.average(),
                           lineSensorB3.average());
}

long SensorController::getUltrasonicDistance() { return hc.dist(); }

long SensorController::getUltrasonicMemory() { return ultrasonicMemory; }

void SensorController::setUltrasonicMemory(long value) {
  ultrasonicMemory = value;
}

int SensorController::getLineSensorAThreshold() { return lineSensorAThreshold; }

void SensorController::setLineSensorAThreshold(int threshold) {
  lineSensorAThreshold = threshold;
}

// ====== ENCODER MEMORY FUNCTIONS =====
void SensorController::setEncoderALastValue(long value) {
  encoderALastValue = value;
}

long SensorController::getEncoderALastValue() { return encoderALastValue; }

void SensorController::setEncoderALastTime(long value) {
  encoderALastTime = value;
}

long SensorController::getEncoderALastTime() { return encoderALastTime; }

void SensorController::setEncoderBLastValue(long value) {
  encoderBLastValue = value;
}

long SensorController::getEncoderBLastValue() { return encoderBLastValue; }

void SensorController::setEncoderBLastTime(long value) {
  encoderBLastTime = value;
}

long SensorController::getEncoderBLastTime() { return encoderBLastTime; }

// ===== ENCODER SPEED STUFF =====

float SensorController::getEncoderASpeed() {
  unsigned long currentTime = millis();
  long encoderReading = encoderA.read();
  float deltaTime = (currentTime - encoderALastTime) / 1000.0;
  long deltaDistance = encoderReading - encoderALastValue;

  setEncoderALastTime(currentTime);
  setEncoderALastValue(encoderReading);

  return deltaDistance / deltaTime;
}

float SensorController::getEncoderBSpeed() {
  unsigned long currentTime = millis();
  long encoderReading = encoderB.read();
  float deltaTime = (currentTime - encoderBLastTime) / 1000.0;
  long deltaDistance = encoderReading - encoderBLastValue;

  setEncoderBLastTime(currentTime);
  setEncoderBLastValue(encoderReading);

  return deltaDistance / deltaTime;
}

float SensorController::speedAdjust(int speedReading, float constraintValue) {
  float quotient = speedReading / ENCODER_MAX_SPEED;
  if (quotient > 1) {
    return constraintValue;
  } else if (quotient < -1) {
    return -constraintValue;
  } else {
    return constraintValue * speedReading / ENCODER_MAX_SPEED;
  }
}