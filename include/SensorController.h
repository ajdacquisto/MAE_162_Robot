#ifndef SENSORCONTROLLER_H
#define SENSORCONTROLLER_H

#include "Encoder.h"
#include "UltrasonicHandler.h"
#include "SimpleSensor.h"
#include "config.h"
#include <Arduino.h>
#include "NewIRSensor.h"

class SensorController {
public:
  // Constructor
  SensorController();

  // Destructor
  ~SensorController();

  enum BUTTON_STATE { PRESSED = HIGH, UNPRESSED = LOW };
  enum LED_STATE { OFF = LOW, ON = HIGH };

private:
  NewIRSensor newIR; // IR sensor
  UltrasonicHandler ultrasonicHandler; // Ultrasonic sensor

  Encoder encFrontRight;
  Encoder encFrontLeft;
  Encoder encRearRight;
  Encoder encRearLeft;

  int lineSensorAThreshold;
  int lineSensorBThreshold;

  // Encoder values
  long encFrontRightLastVal = 0;
  long encFrontLeftLastVal = 0;
  long encRearRightLastVal = 0;
  long encRearLeftLastVal = 0;

  long encFrontRightLastTime = 0;
  long encFrontLeftLastTime = 0;
  long encRearRightLastTime = 0;
  long encRearLeftLastTime = 0;

  LED_STATE currentLEDstate = OFF;

public:
  SimpleSensor lineSensorA1; // Line sensor A1
  SimpleSensor lineSensorA2; // Line sensor A2
  SimpleSensor lineSensorA3; // Line sensor A3
  SimpleSensor lineSensorB1; // Line sensor B1
  SimpleSensor lineSensorB2; // Line sensor B2
  SimpleSensor lineSensorB3; // Line sensor B3

  void zeroEncoder(Encoder &encoder);
  void zeroAllEncoders();
  long readEncoder(Encoder &encoder);

  float getEncFrontRightSpeed();
  float getEncFrontLeftSpeed();
  float getEncRearRightSpeed();
  float getEncRearLeftSpeed();

  int determineError(int lineSensorValue);

  void readLineSensorA();
  void readLineSensorB();

  int getLineSensorAThreshold();
  void setLineSensorAThreshold(int threshold);
  int combineLineResult(int avg1, int avg2, int avg3, int avg4, int avg5,
                        int avg6);

  int getLineSensorBThreshold();
  void setLineSensorBThreshold(int threshold);

  int processBinaryNumber(int binaryNumber[]);
  void intToBinaryArray(int num, int binaryArray[]);

  BUTTON_STATE readButton();

  void init();

  void turnLED(LED_STATE state);
  bool buttonCheck();

  unsigned long getLastUltrasonicRead();

  int getFullIRReadingResults(bool printResults, bool isNewIR);

  NewIRSensor getNewIRSensor();
  UltrasonicHandler getUltrasonicHandler();
};

#endif // SENSORCONTROLLER_H