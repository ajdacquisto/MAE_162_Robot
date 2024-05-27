#ifndef SENSORCONTROLLER_H
#define SENSORCONTROLLER_H

#include "Encoder.h"
#include "HCSR04.h"
#include "MovingAverageSensor.h"
#include "config.h"

class SensorController {
private:
  Encoder encoderA;                 // Encoder A
  Encoder encoderB;                 // Encoder B
  HCSR04 hc;                        // Ultrasonic sensor
  MovingAverageSensor lineSensorA1; // Line sensor A1
  MovingAverageSensor lineSensorA2; // Line sensor A2
  MovingAverageSensor lineSensorA3; // Line sensor A3
  MovingAverageSensor lineSensorB1; // Line sensor B1
  MovingAverageSensor lineSensorB2; // Line sensor B2
  MovingAverageSensor lineSensorB3; // Line sensor B3

  int combineLineResult(int avg1, int avg2, int avg3);
  long ultrasonicMemory;
  int lineSensorAThreshold;
public:
  // Constructor
  SensorController();

  // Destructor
  ~SensorController();

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

  int readLineSensorA1();
  int readLineSensorA2();
  int readLineSensorA3();
  int readLineSensorB1();
  int readLineSensorB2();
  int readLineSensorB3();

  int getLineSensorAThreshold();
  void setLineSensorAThreshold(int threshold);
};

#endif // SENSORCONTROLLER_H