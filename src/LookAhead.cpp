#include "LookAhead.h"

/**
 * @brief Initializes a new instance of the LookAhead class.
 *
 * This constructor is called when a new LookAhead object is created. It prints
 * a message to the serial monitor indicating that the LookAhead object has been
 * initialized.
 */
LookAhead::LookAhead() { Serial.println("LookAhead initialized"); }

/**
 * @brief Destructor for the LookAhead class.
 *
 * This destructor is responsible for destroying an instance of the LookAhead
 * class. It prints a message to the serial monitor indicating that the
 * LookAhead object has been destroyed.
 */
LookAhead::~LookAhead() { Serial.println("LookAhead destroyed"); }

/**
 * Initializes the LookAhead module.
 * This function is called to initialize the LookAhead module.
 * It prints a message to the serial monitor indicating that the initialization
 * has been called.
 */
void LookAhead::init() { Serial.println("LookAhead init called"); }

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

  Serial.print("PID: output(");
  Serial.print(output);
  Serial.print(") = kp(");
  Serial.print(m_kp);
  Serial.print(") * error(");
  Serial.print(error);
  Serial.print(") + ki(");
  Serial.print(m_ki);
  Serial.print(") * integral(");
  Serial.print(m_integral);
  Serial.print(") + kd(");
  Serial.print(m_kd);
  Serial.print(") * derivative(");
  Serial.print(derivative);
  Serial.println(")");

  return output;
}

/**
 * Converts the given line sensor value to a uint8_t value.
 *
 * @param lineSensorValue The line sensor value to be converted.
 * @return The converted uint8_t value.
 */
uint8_t LookAhead::convertToUint8_t(int lineSensorValue, bool useNewIR) {

  Serial.print("Converting line sensor value: ");
  Serial.println(lineSensorValue, BIN);

  // Ensure that the lineSensorValue is within the range of 0-63 (6 bits)
  lineSensorValue &= 0x3F; // 0x3F is 00111111 in binary, masking the six
                           // least significant bits

  // Cast the result to uint8_t
  uint8_t result = static_cast<uint8_t>(lineSensorValue);
  Serial.print("Converted value: ");

  if (!useNewIR) {
    uint8_t leastSignificantBits =
        result & 0x3F; // Masking to get the 6 least significant bits
    for (int i = 5; i >= 0; i--) { // Loop to print each bit from MSB to LSB
      Serial.print((leastSignificantBits >> i) & 0x01);
    }
  } else {
    uint8_t leastSignificantBits =
        result & 0x1F; // Masking to get the 5 least significant bits
    for (int i = 4; i >= 0; i--) { // Loop to print each bit from MSB to LSB
      Serial.print((leastSignificantBits >> i) & 0x01);
    }
  }

  Serial.println();
  return result;
}

/**
 * Adds a sensor reading to the LookAhead buffer.
 *
 * @param sensor_reading The sensor reading to be added.
 */
void LookAhead::addSensorReading(uint8_t sensor_reading, bool useNewIR) {
  Serial.print("Adding sensor reading: ");

  if (!useNewIR) {
    uint8_t leastSignificantBits =
        sensor_reading & 0x3F; // Masking to get the 6 least significant bits
    for (int i = 5; i >= 0; i--) { // Loop to print each bit from MSB to LSB
      Serial.print((leastSignificantBits >> i) & 0x01);
    }
  } else {
    uint8_t leastSignificantBits =
        sensor_reading & 0x1F; // Masking to get the 5 least significant bits
    for (int i = 4; i >= 0; i--) { // Loop to print each bit from MSB to LSB
      Serial.print((leastSignificantBits >> i) & 0x01);
    }
  }

  Serial.println();

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
void LookAhead::getPoints(float points[][2], int &num_points, bool useNewIR) {
  num_points = 0;
  for (int i = 0; i < bufferCount; ++i) {
    int index = (bufferIndex - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
    uint8_t sensor_reading = buffer[index];

    // Skip all-zero readings
    if (sensor_reading == 0) {
      Serial.print("Buffer \t[");
      Serial.print(index);
      Serial.print("] \t= ");
      if (!useNewIR) {
        uint8_t leastSignificantBits =
            sensor_reading &
            0x3F; // Masking to get the 6 least significant bits
        for (int i = 5; i >= 0; i--) { // Loop to print each bit from MSB to LSB
          Serial.print((leastSignificantBits >> i) & 0x01);
        }
      } else {
        uint8_t leastSignificantBits =
            sensor_reading &
            0x1F; // Masking to get the 5 least significant bits
        for (int i = 4; i >= 0; i--) { // Loop to print each bit from MSB to LSB
          Serial.print((leastSignificantBits >> i) & 0x01);
        }
      }
      Serial.println(" -> skipped");
      continue;
    }

    float x_position = calculateXPosition(sensor_reading, useNewIR);
    float y_position = -i;

    points[num_points][0] = x_position;
    points[num_points][1] = y_position;
    num_points++;

    Serial.print("Buffer \t[");
    Serial.print(index);
    Serial.print("] \t= ");
    if (!useNewIR) {
      uint8_t leastSignificantBits =
          sensor_reading & 0x3F; // Masking to get the 6 least significant bits
      for (int i = 5; i >= 0; i--) { // Loop to print each bit from MSB to LSB
        Serial.print((leastSignificantBits >> i) & 0x01);
      }
    } else {
      uint8_t leastSignificantBits =
          sensor_reading & 0x1F; // Masking to get the 5 least significant bits
      for (int i = 4; i >= 0; i--) { // Loop to print each bit from MSB to LSB
        Serial.print((leastSignificantBits >> i) & 0x01);
      }
    }

    Serial.print(" -> (");
    Serial.print(x_position);
    Serial.print(", ");
    Serial.print(y_position);
    Serial.println(")");
  }

  // Serial.print("Number of points retrieved: ");
  // Serial.println(num_points);
}

/**
 * Calculate the x-position for a given sensor reading.
 *
 * @param sensor_reading The sensor reading value.
 * @return The x-position.
 */
float LookAhead::calculateXPosition(uint8_t sensor_reading, bool useNewIR) {
  if (!useNewIR) {
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
  } else {
    const float positions[5] = {-2.0, -1.0, 0.0, 1.0, 2.0};
    int count_ones = 0;
    float sum_positions = 0.0;

    for (int i = 0; i < 5; ++i) {
      if (sensor_reading & (1 << (4 - i))) {
        count_ones++;
        sum_positions += positions[i];
      }
    }

    if (count_ones > 3) {
      return sum_positions / count_ones;
    } else if (count_ones == 3) {
      return sum_positions / count_ones;
    } else if (count_ones == 2) {
      if (sensor_reading & 0b10000) {
        sum_positions += positions[0] - 1.0;
        count_ones++;
      } else if (sensor_reading & 0b00001) {
        sum_positions += positions[4] + 1.0;
        count_ones++;
      }
      return sum_positions / count_ones;
    } else if (count_ones == 1) {
      if (sensor_reading & 0b10000) {
        sum_positions += positions[0] - 1.0;
        sum_positions += positions[1] - 1.0;
        count_ones += 2;
      } else if (sensor_reading & 0b00001) {
        sum_positions += positions[4] + 1.0;
        sum_positions += positions[3] + 1.0;
        count_ones += 2;
      }
      return sum_positions / count_ones;
    }

    return 0.0;
  }
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
bool LookAhead::linearRegression(const float points[][2], int num_points,
                                 int recent_points, float &slope,
                                 float &intercept) {

  if (recent_points > num_points) {
    recent_points = num_points;
  }

  Serial.print("Performing linear regression with ");
  Serial.print(recent_points);
  Serial.println(" recent points");

  if (recent_points == 0) {
    slope = 0;
    intercept = 0;
    Serial.println("<!> No recent points, slope and intercept set to 0");
    return false;
  }

  float sum_x = 0;
  for (int i = 0; i < recent_points; ++i) {
    sum_x += points[i][0];
  }

  float mean_x = sum_x / recent_points;
  bool isVerticalLine = true;
  for (int i = 0; i < recent_points; ++i) {
    if (points[i][0] != mean_x) {
      isVerticalLine = false;
      break;
    }
  }

  if (isVerticalLine) {
    // Handle vertical line case
    slope = INFINITY_VALUE;
    Serial.println("Detected vertical line case");
    if (isnan(mean_x)) {
      Serial.println("Mean x is NaN");
      intercept = 0;
      return true;
    }
    intercept = mean_x; // Can use intercept to store x-value of the line
    return true;
  }

  // Continue with regular linear regression calculation
  float sum_y = 0;
  float sum_xy = 0;
  float sum_xx = 0;

  for (int i = 0; i < recent_points; ++i) {
    float x = points[i][0];
    float y = points[i][1];
    sum_y += y;
    sum_xy += x * y;
    sum_xx += x * x;
  }

  Serial.print("float n = static_cast<float>(");
  Serial.print(recent_points);
  Serial.print("); -> ");

  float n = static_cast<float>(recent_points);

  Serial.println(n);

  float denominator = n * sum_xx - sum_x * sum_x;
  if (denominator == 0) {
    slope = 0;
    intercept = 0;
    Serial.println("<!> Denominator is zero, slope and intercept set to 0");
    return true;
  }

  slope = (n * sum_xy - sum_x * sum_y) / denominator;

  intercept = (sum_y * sum_xx - sum_x * sum_xy) / denominator;

  Serial.print("Calculated line: y = (");
  Serial.print(slope);
  Serial.print(")x + ");
  Serial.println(intercept);

  return true;
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
  if (slope == INFINITY_VALUE) {
    // Handle vertical line case by returning the x-location stored in intercept
    Serial.print("Vertical line at x = ");
    Serial.println(intercept);
    return intercept;
  } else if (slope == 0) {
    Serial.print("Slope is zero, returning intercept: x = ");
    Serial.println(intercept);
    return intercept;
  }
  float predicted_x = (y - intercept) / slope;
  Serial.print("Look ahead point: (x, y) = (");
  Serial.print(predicted_x);
  Serial.print(", ");
  Serial.print(y);
  Serial.println(")");
  return predicted_x;
}
