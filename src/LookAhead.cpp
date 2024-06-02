#include "LookAhead.h"

// Constructor
LookAhead::LookAhead() { Serial.println("LookAhead initialized"); }

// Destructor
LookAhead::~LookAhead() { Serial.println("LookAhead destroyed"); }

// Initialize the LookAhead object
void LookAhead::init() { Serial.println("LookAhead initialized"); }

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

// Intermediate function to convert the combined line result to uint8_t
uint8_t LookAhead::convertToUint8_t(int lineSensorValue) {
  // Ensure that the lineSensorValue is within the range of 0-63 (6 bits)
  lineSensorValue &= 0x3F; // 0x3F is 00111111 in binary, masking the six least
                           // significant bits

  // Cast the result to uint8_t
  return static_cast<uint8_t>(lineSensorValue);
}

// Add a new sensor reading to the buffer
void LookAhead::addSensorReading(uint8_t sensor_reading) {
  buffer[bufferIndex] = sensor_reading;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferCount < BUFFER_SIZE) {
    bufferCount++;
  }
}

/**
 * Get the (X, Y) points from the buffer.
 *
 * @param points The array to store the (X, Y) points.
 * @param num_points The number of points to retrieve.
 */
void LookAhead::getPoints(float points[][2], int &num_points) {
  num_points = 0;
  for (int i = 0; i < bufferCount; ++i) {
    int index = (bufferIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
    uint8_t sensor_reading = buffer[index];

    // Skip all-zero readings
    if (sensor_reading == 0)
      continue;

    float x_position = calculateXPosition(sensor_reading);
    float y_position = -i;

    points[num_points][0] = x_position;
    points[num_points][1] = y_position;
    num_points++;
  }
}

/**
 * Calculate the x-position for a given sensor reading.
 *
 * @param sensor_reading The sensor reading value.
 * @return The x-position.
 */
float LookAhead::calculateXPosition(uint8_t sensor_reading) {
  const float positions[6] = {-2.5, -1.5, -0.5, 0.5, 1.5, 2.5};
  int count_ones = 0;
  float sum_positions = 0.0;

  for (int i = 0; i < 6; ++i) {
    if (sensor_reading & (1 << (5 - i))) {
      count_ones++;
      sum_positions += positions[i];
    }
  }

  if (count_ones > 3) {
    return sum_positions / count_ones;
  } else if (count_ones == 3) {
    return sum_positions / count_ones;
  } else if (count_ones == 2) {
    if (sensor_reading & 0b100000) {
      sum_positions += positions[0] - 1.0;
      count_ones++;
    } else if (sensor_reading & 0b000001) {
      sum_positions += positions[5] + 1.0;
      count_ones++;
    }
    return sum_positions / count_ones;
  } else if (count_ones == 1) {
    if (sensor_reading & 0b100000) {
      sum_positions += positions[0] - 1.0;
      sum_positions += positions[1] - 1.0;
      count_ones += 2;
    } else if (sensor_reading & 0b000001) {
      sum_positions += positions[5] + 1.0;
      sum_positions += positions[4] + 1.0;
      count_ones += 2;
    }
    return sum_positions / count_ones;
  }

  return 0.0;
}

/**
 * Calculate linear regression for the given points.
 *
 * @param points The array of points.
 * @param num_points The number of points.
 * @param recent_points The number of recent points to consider for regression.
 * @param slope The calculated slope of the regression line.
 * @param intercept The calculated intercept of the regression line.
 */
void LookAhead::linearRegression(const float points[][2], int num_points,
                                 int recent_points, float &slope,
                                 float &intercept) {
  if (recent_points > num_points) {
    recent_points = num_points;
  }

  float sum_x = 0;
  float sum_y = 0;
  float sum_xy = 0;
  float sum_xx = 0;

  for (int i = 0; i < recent_points; ++i) {
    float x = points[i][0];
    float y = points[i][1];
    sum_x += x;
    sum_y += y;
    sum_xy += x * y;
    sum_xx += x * x;
  }

  float n = static_cast<float>(recent_points);
  float denominator = n * sum_xx - sum_x * sum_x;
  if (denominator == 0) {
    slope = 0;
    intercept = 0;
    return;
  }

  slope = (n * sum_xy - sum_x * sum_y) / denominator;
  intercept = (sum_y * sum_xx - sum_x * sum_xy) / denominator;
}

/**
 * Predict the x-value for a given y-value using the regression line.
 *
 * @param slope The slope of the regression line.
 * @param intercept The intercept of the regression line.
 * @param y The y-value.
 * @return The predicted x-value.
 */
float LookAhead::predictX(float slope, float intercept, float y) {
  return (y - intercept) / slope;
}
