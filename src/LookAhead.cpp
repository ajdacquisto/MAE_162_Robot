#include "LookAhead.h"

// Constructor
LookAhead::LookAhead() {
  Serial.println("LookAhead initialized");
}

// Destructor
LookAhead::~LookAhead() {
  Serial.println("LookAhead destroyed");
}

void LookAhead::init() {
  Serial.println("LookAhead initialized");
}

/**
 * Collect sensor data and store it in a buffer.
 *
 * @param newSensorData The new sensor data to be collected.
 */
void LookAhead::collectSensorData(int newSensorData) {
  for (int i = numRows - 1; i > 0; i--) {
    for (int j = 0; j < LA_NUM_SENSORS; j++) {
      sensorData[i][j] = sensorData[i - 1][j]; // Copy previous row to the current row
    }
  }

  // Store the new sensor data in the first row (convert newSensorData to array, reversed order)
  for (int j = 0; j < LA_NUM_SENSORS; j++) {
    sensorData[0][j] = (newSensorData >> (LA_NUM_SENSORS - 1 - j)) & 1;
  }

  // Print the sensor data buffer
  Serial.println("Sensor data buffer:");
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
      if (i < 3) {
        linePosition -= (3 - i);
      } else {
        linePosition += (i - 2);
      }
    }
  }

  if (numActiveSensors == 0) {
    return 0; // No line detected
  }

  if (numActiveSensors < 3) {
    int adjustedLinePosition = 0;
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
      if ((sensorReading >> i) & 1) {
        if (i < 3) {
          adjustedLinePosition -= (3 - i);
        } else {
          adjustedLinePosition += (i - 2);
        }
      }
    }
    return adjustedLinePosition / 3; // Average position for assumed width of 3
  }

  return linePosition / numActiveSensors;
}

/**
 * Check if the points form a straight line.
 *
 * @param points The array of points.
 * @param numPoints The number of points.
 * @return True if the points form a straight line, false otherwise.
 */
bool LookAhead::isStraightLine(Point points[], int numPoints) {
  if (numPoints < 2) {
    return true; // Not enough points to determine curvature
  }

  float initialSlope = (points[1].y - points[0].y) / (points[1].x - points[0].x);

  for (int i = 2; i < numPoints; i++) {
    float slope = (points[i].y - points[i - 1].y) / (points[i].x - points[i - 1].x);
    if (fabs(slope - initialSlope) > 0.01) { // Allow some tolerance for straightness
      return false;
    }
  }
  return true;
}

/**
 * Use Euler method to follow a straight line.
 *
 * @param points The array of points.
 * @param numPoints The number of points.
 * @param xTarget The x-value for interpolation.
 * @return The interpolated point.
 */
LookAhead::Point LookAhead::eulerMethod(Point points[], int numPoints, float xTarget) {
  float slope = (points[numPoints - 1].y - points[0].y) / (points[numPoints - 1].x - points[0].x);
  float intercept = points[0].y - slope * points[0].x;

  float yTarget = slope * xTarget + intercept;

  Serial.print("Euler Method Point at x=");
  Serial.print(xTarget);
  Serial.print(": (");
  Serial.print(xTarget);
  Serial.print(", ");
  Serial.print(yTarget);
  Serial.println(")");

  return {xTarget, yTarget};
}

/**
 * Interpolate a point on a spline curve using all points.
 *
 * @param points The array of points.
 * @param numPoints The number of points.
 * @param xTarget The x-value for interpolation.
 * @return The interpolated point.
 */
LookAhead::Point LookAhead::interpolateSpline(Point points[], int numPoints, float xTarget) {
  if (numPoints < 2) {
    Serial.println("Error: Not enough points for interpolation");
    return {0, 0};
  }

  // Polynomial regression to fit a curve (swap x and y)
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  for (int i = 0; i < numPoints; i++) {
    sumX += points[i].x;
    sumY += points[i].y;
    sumXY += points[i].x * points[i].y;
    sumX2 += points[i].x * points[i].x;
  }

  float denominator = (numPoints * sumX2 - sumX * sumX);
  if (denominator == 0) {
    Serial.println("Error: denominator is zero");
    return {0, 0};
  }

  float a = (numPoints * sumXY - sumX * sumY) / denominator;
  float b = (sumY * sumX2 - sumX * sumXY) / denominator;

  float yTarget = a * xTarget + b;

  if (isnan(yTarget) || isinf(yTarget)) {
    Serial.println("Error: invalid yTarget");
    return {0, 0};
  }

  Serial.print("Interpolated Point at x=");
  Serial.print(xTarget);
  Serial.print(": (");
  Serial.print(xTarget);
  Serial.print(", ");
  Serial.print(yTarget);
  Serial.println(")");

  return {xTarget, yTarget};
}

/**
 * Get the look-ahead point based on the look-ahead distance.
 *
 * @param lookAheadDistance The look-ahead distance.
 * @return The look-ahead point.
 */
LookAhead::Point LookAhead::getLookAheadPoint(float lookAheadDistance) {
  Point points[numRows];
  int validRowCount = 0;

  for (int i = 0; i < numRows; i++) {
    int sensorReading = 0;
    bool allZeros = true;

    for (int j = 0; j < LA_NUM_SENSORS; j++) {
      sensorReading |= (sensorData[i][j] << j);
      if (sensorData[i][j] != 0) {
        allZeros = false;
      }
    }

    if (!allZeros) {
      int linePosition = calculateLinePosition(sensorReading);
      points[validRowCount].x = (float)i;  // Use positive values for x
      points[validRowCount].y = (float)linePosition;
      validRowCount++;
    }
  }

  Serial.println("Points used for interpolation:");
  for (int i = 0; i < validRowCount; i++) {
    Serial.print("Point ");
    Serial.print(i);
    Serial.print(": (");
    Serial.print(points[i].x);
    Serial.print(", ");
    Serial.print(points[i].y);
    Serial.println(")");
  }

  if (validRowCount < 2) {
    Serial.println("Not enough valid points for interpolation");
    return {0, 0}; // Return a default point
  }

  LookAhead::Point lookAheadPoint;

  if (isStraightLine(points, validRowCount)) {
    lookAheadPoint = eulerMethod(points, validRowCount, 5.0);  // Change to x=5
  } else {
    lookAheadPoint = interpolateSpline(points, validRowCount, 5.0);  // Change to x=5
  }

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
