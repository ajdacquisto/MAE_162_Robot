#ifndef SENSORCONTROLLER_H
#define SENSORCONTROLLER_H

#include "Encoder.h"
#include "HCSR04.h"
#include "MovingAverageSensor.h"
#include "config.h"

class SensorController {
private:
  Encoder encoderA; // Encoder A
  Encoder encoderB; // Encoder B
  HCSR04 hc;        // Ultrasonic sensor

  long ultrasonicMemory;
  int lineSensorAThreshold;
  int lineSensorBThreshold;

  long encoderALastValue = 0;
  long encoderALastTime = 0;
  long encoderBLastValue = 0;
  long encoderBLastTime = 0;

public:
  // Constructor
  SensorController();

  // Destructor
  ~SensorController();

  enum BUTTON_STATE { PRESSED = HIGH, UNPRESSED = LOW };

  MovingAverageSensor lineSensorA1; // Line sensor A1
  MovingAverageSensor lineSensorA2; // Line sensor A2
  MovingAverageSensor lineSensorA3; // Line sensor A3
  MovingAverageSensor lineSensorB1; // Line sensor B1
  MovingAverageSensor lineSensorB2; // Line sensor B2
  MovingAverageSensor lineSensorB3; // Line sensor B3

  void zeroEncoders();
  long getUltrasonicDistance();
  long readEncoderA();
  long readEncoderB();
  int determineError(int lineSensorValue);

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
};

#endif // SENSORCONTROLLER_H