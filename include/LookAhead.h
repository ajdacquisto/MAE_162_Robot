#ifndef LOOKAHEAD_H
#define LOOKAHEAD_H

#include "config.h"
#include <math.h>
#include <Arduino.h>

class LookAhead {
public:
  LookAhead();
  ~LookAhead();

  struct Point {
    float x, y;
  };

  // Add your member functions here
  void init();

  void collectSensorData(int newSensorData);
  Point getLookAheadPoint(float lookAheadDistance);
  float PID(float error);

#ifdef UNIT_TEST
public:
#else
private:
#endif
  // Variables
  const int m_sensorWeights[LA_NUM_SENSORS] = {-3, -2, -1, 1, 2, 3};
  const float m_kp = 20.0, m_ki = 0.5, m_kd = 0.0;
  float m_lastError = 0, m_integral = 0;

  int numRows = 10; // Tunable value for number of rows to use
  int sensorData[LA_MAX_ROWS][LA_NUM_SENSORS]; // Properly declared array

  // Methods
  int calculateLinePosition(int sensorReading);
  Point interpolateSpline(Point points[], int numPoints, float t);
  Point eulerMethod(Point points[], int numPoints, float xTarget);
  bool isStraightLine(Point points[], int numPoints);
};

#endif // LOOKAHEAD_H
