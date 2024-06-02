#ifndef LOOKAHEAD_H
#define LOOKAHEAD_H

#include "config.h"
#include <Arduino.h>
#include <math.h>
#include <stdint.h>

class LookAhead {
public:
  LookAhead();
  ~LookAhead();

  // Add your member functions here
  void init();
  float PID(float error);
  uint8_t convertToUint8_t(int lineSensorValue);
  void addSensorReading(uint8_t sensor_reading);
  void getPoints(float points[][2], int &num_points);
  void linearRegression(const float points[][2], int num_points,
                        int recent_points, float &slope, float &intercept);

  float predictX(float slope, float intercept, float y);

#ifdef UNIT_TEST
public:
#else
private:
#endif
  // Variables
  const float m_kp = 20.0, m_ki = 0.5, m_kd = 0.0;
  float m_lastError = 0, m_integral = 0;

  static const int BUFFER_SIZE = 10;
  uint8_t buffer[BUFFER_SIZE];
  int bufferIndex;
  int bufferCount;

  // Functions
  float calculateXPosition(uint8_t sensor_reading);
};

#endif // LOOKAHEAD_H
