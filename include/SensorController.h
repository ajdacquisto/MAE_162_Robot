#ifndef SENSORCONTROLLER_H
#define SENSORCONTROLLER_H

#include "Encoder.h"
#include "HCSR04.h"
#include "SimpleSensor.h"
#include "config.h"
#include <Arduino.h>

class SensorController {
public:
  // Constructor
  SensorController();

  // Destructor
  ~SensorController();

  enum BUTTON_STATE { PRESSED = HIGH, UNPRESSED = LOW };
  enum LED_STATE { OFF = LOW, ON = HIGH };

private:
  Encoder encoderA; // Encoder A
  Encoder encoderB; // Encoder B
  HCSR04 hc;        // Ultrasonic sensor

  long ultrasonicMemory;
  int lineSensorAThreshold;
  int lineSensorBThreshold;
  long previousDistance = 0;
  long currentDistance = 0;

  long encoderALastValue = 0;
  long encoderALastTime = 0;
  long encoderBLastValue = 0;
  long encoderBLastTime = 0;

  LED_STATE currentLEDstate = OFF;

  unsigned long lastUltrasonicRead = 0;

public:
  SimpleSensor lineSensorA1; // Line sensor A1
  SimpleSensor lineSensorA2; // Line sensor A2
  SimpleSensor lineSensorA3; // Line sensor A3
  SimpleSensor lineSensorB1; // Line sensor B1
  SimpleSensor lineSensorB2; // Line sensor B2
  SimpleSensor lineSensorB3; // Line sensor B3

  void zeroEncoders();
  long getUltrasonicDistance();
  long readEncoderA();
  long readEncoderB();
  int determineError(int lineSensorValue);

  long getPreviousDistance();

  void readLineSensorA();
  void readLineSensorB();

  long getUltrasonicMemory();
  void setUltrasonicMemory(long value);

  int getLineSensorAThreshold();
  void setLineSensorAThreshold(int threshold);
  int combineLineResult(int avg1, int avg2, int avg3, int avg4, int avg5,
                        int avg6);

  int getLineSensorBThreshold();
  void setLineSensorBThreshold(int threshold);

  long getEncoderALastValue();
  long getEncoderALastTime();
  long getEncoderBLastValue();
  long getEncoderBLastTime();
  void setEncoderALastValue(long value);
  void setEncoderALastTime(long value);
  void setEncoderBLastValue(long value);
  void setEncoderBLastTime(long value);

  float getEncoderASpeed();
  float getEncoderBSpeed();
  float speedAdjust(int speedReading, float constraintValue);

  bool isObstacle(long distanceThreshold);

  int processBinaryNumber(int binaryNumber[]);
  void intToBinaryArray(int num, int binaryArray[]);

  BUTTON_STATE readButton();

  void init();

  void turnLED(LED_STATE state);
  bool buttonCheck();

  unsigned long getLastUltrasonicRead();

  int getFullIRReadingResults(bool printResults);
};

#endif // SENSORCONTROLLER_H