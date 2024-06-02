#include "LookAhead.h"
#include <Arduino.h>

// Constructor
LookAhead::LookAhead() {
  // Initialize member variables
  Serial.println("LookAhead initialized");
}

// Destructor
LookAhead::~LookAhead() {
  // Clean up any resources
  Serial.println("LookAhead destroyed");
}

void LookAhead::init() { Serial.println("LookAhead initialized"); }

/**
 * Collect sensor data and store it in a buffer.
 *
 * @param newSensorData The new sensor data to be collected.
 */
void LookAhead::collectSensorData(int newSensorData) {
  Serial.println("Collecting sensor data:");
  for (int i = numRows - 1; i > 0; i--) {
    for (int j = 0; j < LA_NUM_SENSORS; j++) {
      sensorData[i][j] =
          sensorData[i - 1][j]; // Copy previous row to the current row
    }
  }

  // Store the new sensor data in the first row (convert newSensorData to array,
  // reversed order)
  for (int j = 0; j < LA_NUM_SENSORS; j++) {
    sensorData[0][j] = (newSensorData >> (LA_NUM_SENSORS - 1 - j)) & 1;
  }

  // Print the sensor data buffer
  for (int i = 0; i < numRows; i++) {
    Serial.print("Row ");
    Serial.print(i);
    Serial.print(": ");
    for (int j = 0; j < LA_NUM_SENSORS; j++) {
      Serial.print(sensorData[i][j], BIN);
      Serial.print(" ");
    }
    Serial.println();
  }
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
    Serial.println("No line detected");
    return 0; // No line detected
  }

  int calculatedPosition = linePosition / numActiveSensors;
  Serial.print("Sensor Reading: ");
  Serial.print(sensorReading, BIN);
  Serial.print(" Calculated Line Position: ");
  Serial.println(calculatedPosition);
  return calculatedPosition;
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
  Serial.print("Interpolated Point: (");
  Serial.print(result.x);
  Serial.print(", ");
  Serial.print(result.y);
  Serial.println(")");
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
    int linePosition = 0;
    int numActiveSensors = 0;
    for (int j = 0; j < LA_NUM_SENSORS; j++) {
      if (sensorData[i][j] == 1) {
        numActiveSensors++;
        if (j <= 2) {
          linePosition -= (3 - j);
        } else {
          linePosition += (j - 2);
        }
      }
    }

    // Calculate the average position if there are active sensors
    if (numActiveSensors > 0) {
      points[i].x = (float)linePosition / numActiveSensors;
    } else {
      points[i].x = 0; // Default to 0 if no sensors are active
    }
    points[i].y = i; // Use row index as y-coordinate

    Serial.print("Point ");
    Serial.print(i);
    Serial.print(": (");
    Serial.print(points[i].x);
    Serial.print(", ");
    Serial.print(points[i].y);
    Serial.println(")");
  }

  // Interpolate spline to find look-ahead point
  Point lookAheadPoint;
  float t = lookAheadDistance / numRows; // Normalize t to the range [0, 1]
  lookAheadPoint = interpolateSpline(points[0], points[1], points[2],
                                     points[numRows - 1], t);

  Serial.print("Look-Ahead Point: (");
  Serial.print(lookAheadPoint.x);
  Serial.print(", ");
  Serial.print(lookAheadPoint.y);
  Serial.println(")");

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

  Serial.print("PID Error: ");
  Serial.print(error);
  Serial.print(" Output: ");
  Serial.println(output);

  return output;
}