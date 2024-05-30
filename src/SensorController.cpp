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
  lineSensorAThreshold = LINE_SENSOR_THRESHOLD;
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

int SensorController::combineLineResult(int avg1, int avg2, int avg3, int avg4,
                                        int avg5, int avg6) {
  // CONVENTION: 1 = black ON-TARGET, 0 = white OFF-TARGET
  bool DO_READ_ONE_BY_ONE = false;

  int lineSensorValueA1 = (avg1 > LINE_SENSOR_THRESHOLD) ? 1 : 0;
  int lineSensorValueA2 = (avg2 > LINE_SENSOR_THRESHOLD) ? 1 : 0;
  int lineSensorValueA3 = (avg3 > LINE_SENSOR_THRESHOLD) ? 1 : 0;
  int lineSensorValueB1 = (avg4 > LINE_SENSOR_THRESHOLD) ? 1 : 0;
  int lineSensorValueB2 = (avg5 > LINE_SENSOR_THRESHOLD) ? 1 : 0;
  int lineSensorValueB3 = (avg6 > LINE_SENSOR_THRESHOLD) ? 1 : 0;

  if (DO_READ_ONE_BY_ONE) {
    Serial.print(lineSensorValueA1);
    Serial.print(lineSensorValueA2);
    Serial.print(lineSensorValueA3);
    Serial.print(lineSensorValueB1);
    Serial.print(lineSensorValueB2);
    Serial.println(lineSensorValueB3);
  }

  // COMBINE values into one variable (e.g. 000001, 000000, 111111, 101101, etc)
  int lineSensorValue = (lineSensorValueA1 << 5) | (lineSensorValueA2 << 4) |
                        (lineSensorValueA3 << 3) | (lineSensorValueB1 << 2) |
                        (lineSensorValueB2 << 1) | lineSensorValueB3;

  return lineSensorValue;
}

int SensorController::determineError(int lineSensorValue) {
  // Sensor positions (assuming 6 sensors): -3, -2, -1, 1, 2, 3
  static const int sensorPositions[6] = {-3, -1, 0, 0, 1, 3};

  int sumWeightedPositions = 0;
  int sumSensorValues = 0;

  // Iterate through each sensor (from least significant bit to most significant
  // bit)
  for (int i = 0; i < 6; i++) {
    // Check if the sensor i is detecting the line (bit i of lineSensorValue is
    // 1)
    if (lineSensorValue & (1 << i)) {
      sumWeightedPositions += sensorPositions[i];
      sumSensorValues += 1;
    }
  }

  // If no sensors are detecting the line, return a high error value (e.g., 99)
  if (sumSensorValues == 0) {
    return 99;
  }

  // Calculate the average error
  int error = sumWeightedPositions / sumSensorValues;

  return error;
}

long SensorController::getUltrasonicDistance() { return hc.dist(); }

long SensorController::getUltrasonicMemory() { return ultrasonicMemory; }

void SensorController::setUltrasonicMemory(long value) {
  ultrasonicMemory = value;
}

int SensorController::getLineSensorAThreshold() { return lineSensorAThreshold; }

int SensorController::getLineSensorBThreshold() { return lineSensorBThreshold; }

void SensorController::setLineSensorAThreshold(int threshold) {
  lineSensorAThreshold = threshold;
}

void SensorController::setLineSensorBThreshold(int threshold) {
  lineSensorBThreshold = threshold;
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