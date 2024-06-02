#ifndef LOOKAHEAD_H
#define LOOKAHEAD_H

#include "config.h"
#include <Arduino.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>

/**
 * @class LookAhead
 * @brief Class for performing look-ahead calculations and sensor readings.
 */
class LookAhead {
public:
  LookAhead();
  ~LookAhead();

  // Add your member functions here

  /**
   * @brief Initializes the LookAhead object.
   */
  void init();

  /**
   * @brief Performs PID control based on the error value.
   * @param error The error value.
   * @return The PID control output.
   */
  float PID(float error);

  /**
   * @brief Converts an integer line sensor value to an 8-bit unsigned integer.
   * @param lineSensorValue The line sensor value.
   * @return The converted 8-bit unsigned integer value.
   */
  uint8_t convertToUint8_t(int lineSensorValue);

  /**
   * @brief Adds a sensor reading to the buffer.
   * @param sensor_reading The sensor reading to add.
   */
  void addSensorReading(uint8_t sensor_reading);

  /**
   * @brief Retrieves the points stored in the buffer.
   * @param points The array to store the points.
   * @param num_points The number of points retrieved.
   */
  void getPoints(float points[][2], int &num_points);

  /**
   * @brief Performs linear regression on the given points.
   * @param points The array of points.
   * @param num_points The number of points.
   * @param recent_points The number of recent points to consider.
   * @param slope The calculated slope of the regression line.
   * @param intercept The calculated intercept of the regression line.
   */
  void linearRegression(const float points[][2], int num_points,
                        int recent_points, float &slope, float &intercept);

  /**
   * @brief Predicts the x-coordinate based on the given slope, intercept, and
   * y-coordinate.
   * @param slope The slope of the regression line.
   * @param intercept The intercept of the regression line.
   * @param y The y-coordinate.
   * @return The predicted x-coordinate.
   */
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
  float INFINITY_VALUE = 1e30; // Member variable for infinity value

  // Functions
  /**
   * @brief Calculates the x-position based on the given sensor reading.
   * @param sensor_reading The sensor reading.
   * @return The calculated x-position.
   */
  float calculateXPosition(uint8_t sensor_reading);
};

#endif // LOOKAHEAD_H
