#ifndef SENSORCONTROLLER_H
#define SENSORCONTROLLER_H

#include "Encoder.h"
#include "HCSR04.h"
#include "config.h"
#include "MovingAverageSensor.h"

class SensorController {
private:
  Encoder encoderA;                 // Encoder A
  Encoder encoderB;                 // Encoder B
  HCSR04 hc;                        // Ultrasonic sensor

  long ultrasonicMemory;
  int lineSensorAThreshold;

  int encoderALastValue;
  long encoderALastTime;
  int encoderBLastValue;
  long encoderBLastTime;
public:
  // Constructor
  SensorController();

  // Destructor
  ~SensorController();

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

  int getLineResultA();
  int getLineResultB();

  void readLineSensorA();
  void readLineSensorB();

  long getUltrasonicMemory();
  void setUltrasonicMemory(long value);

  int getLineSensorAThreshold();
  void setLineSensorAThreshold(int threshold);
  int combineLineResult(int avg1, int avg2, int avg3);

  int getEncoderALastValue();
  long getEncoderALastTime();
  int getEncoderBLastValue();
  long getEncoderBLastTime();
  void setEncoderALastValue(int value);
  void setEncoderALastTime(long value);
  void setEncoderBLastValue(int value);
  void setEncoderBLastTime(long value);

  float getEncoderASpeed();
  float getEncoderBSpeed();
};

#endif // SENSORCONTROLLER_H