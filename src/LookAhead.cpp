#include "LookAhead.h"

// Constructor
LookAhead::LookAhead() {
  // Initialize member variables
}

// Destructor
LookAhead::~LookAhead() {
  // Clean up any resources
}

void LookAhead::init() {}

/**
 * Collect sensor data and store it in a buffer.
 *
 * @param newSensorData The new sensor data to be collected.
 */
void LookAhead::collectSensorData(int newSensorData) {
  for (int i = numRows - 1; i > 0; i--) {
    for (int j = 0; j < LA_NUM_SENSORS; j++) {
      sensorData[i][j] =
          sensorData[i - 1][j]; // Copy previous row to the current row
    }
  }
  sensorData[0][0] =
      newSensorData; // Store the new sensor data in the first row
}

/**
 * Calculate the line position based on the sensor reading.
 *
 * @param sensorReading The sensor reading.
 * @return The calculated line position.
 */
int LookAhead::calculateLinePosition(int sensorReading) {
  int linePosition = 0;
  int numActiveSensors = 0;

  for (int i = 0; i < LA_NUM_SENSORS; i++) {
    if ((sensorReading >> i) & 1) {
      numActiveSensors++;
      if (i <= 2) {
        linePosition -= (3 - i);
      } else {
        linePosition += (i - 2);
      }
    }
  }

  // Handle edge cases where the line is partially out of sensor range
  if (numActiveSensors == 0) {
    return 0; // No line detected
  }

  return linePosition / numActiveSensors;
}

/**
 * Interpolate a point on a spline curve.
 *
 * @param p0 The first control point.
 * @param p1 The second control point.
 * @param p2 The third control point.
 * @param p3 The fourth control point.
 * @param t The parameter value.
 * @return The interpolated point.
 */
LookAhead::Point LookAhead::interpolateSpline(Point p0, Point p1, Point p2,
                                              Point p3, float t) {
  Point result;
  float t2 = t * t;
  float t3 = t2 * t;
  result.x = 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t +
                    (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
                    (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
  result.y = 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t +
                    (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
                    (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
  return result;
}

/**
 * Get the look-ahead point based on the look-ahead distance.
 *
 * @param lookAheadDistance The look-ahead distance.
 * @return The look-ahead point.
 */
LookAhead::Point LookAhead::getLookAheadPoint(float lookAheadDistance) {
  // Convert sensor data to points
  Point points[numRows];
  for (int i = 0; i < numRows; i++) {
    int linePosition = calculateLinePosition(sensorData[i][0]);
    points[i].x = linePosition;
    points[i].y = i; // Use row index as y-coordinate
  }

  // Interpolate spline to find look-ahead point
  Point lookAheadPoint;
  float t = lookAheadDistance / numRows; // Normalize t to the range [0, 1]
  lookAheadPoint = interpolateSpline(points[0], points[1], points[2],
                                     points[numRows - 1], t);

  return lookAheadPoint;
}

/**
 * Perform PID control based on the error.
 *
 * @param error The error value.
 * @return The output of the PID control.
 */
float LookAhead::PID(float error) {
  m_integral += error;
  float derivative = error - m_lastError;
  float output = m_kp * error + m_ki * m_integral + m_kd * derivative;
  m_lastError = error;
  return output;
}