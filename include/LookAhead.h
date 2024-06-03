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
  bool linearRegression(const float points[][2], int num_points,
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
  /**
   * @brief The proportional gain for the LookAhead controller.
   *
   * This constant represents the proportional gain used in the LookAhead controller.
   * It determines the strength of the control action based on the error between the desired
   * and actual values.
   */
  const float m_kp = LA_KP;

  /**
   * @brief The integral gain for the LookAhead controller.
   *
   * This constant represents the integral gain used in the LookAhead controller.
   * It is typically used to adjust the response of the controller to steady-state errors.
   * The value of this constant is defined as LA_KI.
   */
  const float m_ki = LA_KI;

  /**
   * @brief The derivative gain for the LookAhead controller.
   *
   * This constant represents the derivative gain used in the LookAhead controller.
   * It is typically denoted as KD and is used to adjust the response of the controller
   * to changes in the error rate. The value of this constant is defined as LA_KD.
   */
  const float m_kd = LA_KD;

  /**
   * @brief The last error value.
   *
   * This variable stores the last error value encountered during the execution of the program.
   * It is used for tracking the previous error value and can be helpful in various control algorithms.
   */
  float m_lastError = 0;
  
  /**
   * @brief The integral term used in a control algorithm.
   *
   * This variable represents the accumulated sum of the error over time, which is used in control algorithms
   * such as PID controllers. It helps to correct for steady-state errors and improve the system's response.
   */
  float m_integral = 0;

  /**
   * @brief The size of the buffer used in LookAhead.
   */
  static const int BUFFER_SIZE = 10;

  /**
   * @brief The buffer used for storing data.
   */
  uint8_t buffer[BUFFER_SIZE];

  /**
   * @brief The index of the buffer.
   */
  int bufferIndex;

  /**
   * @brief The number of elements in the buffer.
   */
  int bufferCount;
  
  /**
   * @brief Represents the infinity value used in the LookAhead class.
   *
   * This member variable is used to represent the infinity value in the LookAhead class.
   * It is initialized to a very large floating-point value (1e30) to serve as a placeholder
   * for infinity in calculations.
   */
  float INFINITY_VALUE = 1e30;

  // Functions
  /**
   * @brief Calculates the x-position based on the given sensor reading.
   * @param sensor_reading The sensor reading.
   * @return The calculated x-position.
   */
  float calculateXPosition(uint8_t sensor_reading);
};

#endif // LOOKAHEAD_H
