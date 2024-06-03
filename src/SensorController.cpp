#include "SensorController.h"

// Constructor
SensorController::SensorController()
    : encoderA(ENCODER_PIN_A1, ENCODER_PIN_A2),     // Encoder A
      encoderB(ENCODER_PIN_B1, ENCODER_PIN_B2),     // Encoder B
      hc(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN), // Ultrasonic sensor
      lineSensorA1(LINE_SENSOR_PIN_A1),             // Line sensor A1
      lineSensorA2(LINE_SENSOR_PIN_A2),             // Line sensor A2
      lineSensorA3(LINE_SENSOR_PIN_A3),             // Line sensor A3
      lineSensorB1(LINE_SENSOR_PIN_B1),             // Line sensor B1
      lineSensorB2(LINE_SENSOR_PIN_B2),             // Line sensor B2
      lineSensorB3(LINE_SENSOR_PIN_B3)              // Line sensor B3
{
  ultrasonicMemory = 0;
  lineSensorAThreshold = LINE_SENSOR_THRESHOLD;
}

// Destructor
SensorController::~SensorController() {
  // Clean up any resources
}

// ===== SETUP FUNCTIONS =====
void SensorController::zeroEncoders() {
  encoderA.write(0);
  encoderB.write(0);
}

// ===== SENSOR READINGS =====
long SensorController::readEncoderA() { return encoderA.read(); }

long SensorController::readEncoderB() { return encoderB.read(); }

void SensorController::readLineSensorA() {
  lineSensorA1.read();
  lineSensorA2.read();
  lineSensorA3.read();
}

void SensorController::readLineSensorB() {
  lineSensorB1.read();
  lineSensorB2.read();
  lineSensorB3.read();
}

int SensorController::combineLineResult(int avg1, int avg2, int avg3, int avg4, int avg5, int avg6) {
    // Define the average values for each sensor
    const int AVERAGE_VALUES[6] = {770, 712, 796, 787, 765, 847};

    // Calculate the threshold for each sensor dynamically based on the provided averages
    int lineSensorValueA1 = (avg1 > (AVERAGE_VALUES[0] + 10 + 0*(1000 - AVERAGE_VALUES[0]) / 2)) ? 1 : 0;
    int lineSensorValueA2 = (avg2 > (AVERAGE_VALUES[1] + 25 + 0*(1000 - AVERAGE_VALUES[1]) / 2)) ? 1 : 0;
    int lineSensorValueA3 = (avg3 > (AVERAGE_VALUES[2] - 0 + 0*(1000 - AVERAGE_VALUES[2]) / 2)) ? 1 : 0;
    int lineSensorValueB1 = (avg4 > (AVERAGE_VALUES[3] - 0 + 0*(1000 - AVERAGE_VALUES[3]) / 2)) ? 1 : 0;
    int lineSensorValueB2 = (avg5 > (AVERAGE_VALUES[4] - 0 + 0*(1000 - AVERAGE_VALUES[4]) / 2)) ? 1 : 0;
    int lineSensorValueB3 = (avg6 > (AVERAGE_VALUES[5] - 0 + 0*(1000 - AVERAGE_VALUES[5]) / 2)) ? 1 : 0;

    // Optional: Print the sensor values for debugging
    bool DO_READ_ONE_BY_ONE = false;
    if (DO_READ_ONE_BY_ONE) {
        Serial.print(lineSensorValueA1);
        Serial.print(lineSensorValueA2);
        Serial.print(lineSensorValueA3);
        Serial.print(lineSensorValueB1);
        Serial.print(lineSensorValueB2);
        Serial.println(lineSensorValueB3);
    }

    // Combine values into one variable (e.g. 000001, 000000, 111111, 101101, etc)
    int lineSensorValue = (lineSensorValueA1 << 5) | (lineSensorValueA2 << 4) |
                          (lineSensorValueA3 << 3) | (lineSensorValueB1 << 2) |
                          (lineSensorValueB2 << 1) | lineSensorValueB3;

    return lineSensorValue;
}

int SensorController::determineError(int lineSensorValue) {
  bool DO_NEW_FANCY_MODE = true;
  int error = 0;

  if (DO_NEW_FANCY_MODE) {
    int binaryArray[6];
    intToBinaryArray(lineSensorValue, binaryArray);
    error = processBinaryNumber(binaryArray);
  } else {
    // Sensor positions (assuming 6 sensors): -3, -2, -1, 1, 2, 3
    static const int sensorPositions[6] = {-3, 0, 0, 0, 0, 3};

    int sumWeightedPositions = 0;
    int sumSensorValues = 0;

    // Iterate through each sensor (from least significant bit to most
    // significant bit)
    for (int i = 0; i < 6; i++) {
      // Check if the sensor i is detecting the line (bit i of lineSensorValue
      // is 1)
      if (lineSensorValue & (1 << i)) {
        sumWeightedPositions += sensorPositions[i];
        sumSensorValues += 1;
      }
    }

    // If no sensors are detecting the line, return a high error value (e.g.,
    // 99)
    if (sumSensorValues == 0) {
      return 99;
    }

    // Calculate the average error
    error = sumWeightedPositions / sumSensorValues;
  }

  return error;
}

long SensorController::getUltrasonicDistance() {
  lastUltrasonicRead = millis();
  long output = hc.dist();
  previousDistance = currentDistance;
  currentDistance = output;
  return output;
}

long SensorController::getUltrasonicMemory() { return ultrasonicMemory; }

void SensorController::setUltrasonicMemory(long value) {
  ultrasonicMemory = value;
}

int SensorController::getLineSensorAThreshold() { return lineSensorAThreshold; }

int SensorController::getLineSensorBThreshold() { return lineSensorBThreshold; }

void SensorController::setLineSensorAThreshold(int threshold) {
  lineSensorAThreshold = threshold;
}

void SensorController::setLineSensorBThreshold(int threshold) {
  lineSensorBThreshold = threshold;
}

// ====== ENCODER MEMORY FUNCTIONS =====
void SensorController::setEncoderALastValue(long value) {
  encoderALastValue = value;
}

long SensorController::getEncoderALastValue() { return encoderALastValue; }

void SensorController::setEncoderALastTime(long value) {
  encoderALastTime = value;
}

long SensorController::getEncoderALastTime() { return encoderALastTime; }

void SensorController::setEncoderBLastValue(long value) {
  encoderBLastValue = value;
}

long SensorController::getEncoderBLastValue() { return encoderBLastValue; }

void SensorController::setEncoderBLastTime(long value) {
  encoderBLastTime = value;
}

long SensorController::getEncoderBLastTime() { return encoderBLastTime; }

// ===== ENCODER SPEED STUFF =====

float SensorController::getEncoderASpeed() {
  unsigned long currentTime = millis();
  long encoderReading = encoderA.read();
  float deltaTime = (currentTime - encoderALastTime) / 1000.0;
  long deltaDistance = encoderReading - encoderALastValue;

  setEncoderALastTime(currentTime);
  setEncoderALastValue(encoderReading);

  return deltaDistance / deltaTime;
}

float SensorController::getEncoderBSpeed() {
  unsigned long currentTime = millis();
  long encoderReading = encoderB.read();
  float deltaTime = (currentTime - encoderBLastTime) / 1000.0;
  long deltaDistance = encoderReading - encoderBLastValue;

  setEncoderBLastTime(currentTime);
  setEncoderBLastValue(encoderReading);

  return deltaDistance / deltaTime;
}

float SensorController::speedAdjust(int speedReading, float constraintValue) {
  float quotient = speedReading / ENCODER_MAX_SPEED;
  if (quotient > 1) {
    return constraintValue;
  } else if (quotient < -1) {
    return -constraintValue;
  } else {
    return constraintValue * speedReading / ENCODER_MAX_SPEED;
  }
}

bool SensorController::isObstacle(long distanceThreshold) {
  long distance = getUltrasonicDistance();
  long prevDist = getPreviousDistance();
  Serial.print("Ultrasonic data: (old) ");
  Serial.print(prevDist);
  Serial.print(", (new) ");
  Serial.println(distance);
  if (distance < distanceThreshold &&
      prevDist < distanceThreshold) {
    Serial.print(distance);
    Serial.print(" and ");
    Serial.print(prevDist);
    Serial.println(" are both less than ");
    Serial.println(distanceThreshold);
    return true;
  } else {
    return false;
  }
}

long SensorController::getPreviousDistance() { return previousDistance; }

// Function to process the input binary number and return the desired output
int SensorController::processBinaryNumber(int binaryNumber[]) {
  int sum = 0;
  int count = 0;

  // Calculate sum of positions of ones and count of ones in a single loop
  for (int i = 0; i < 6; i++) {
    if (binaryNumber[i] == 1) {
      sum += i;
      count++;
    }
  }

  // Avoid division by zero if there are no ones
  if (count == 0)
    return -3;

  // Calculate adjusted average position
  float avgPos = (float)sum / count;
  float adjustedAvgPos = avgPos - 3;

  // Convert adjusted average position to the desired output
  int output = adjustedAvgPos;

  if (abs(output) <= 1) {
    output = 0;
  }

  return output;
}

void SensorController::intToBinaryArray(int num, int binaryArray[]) {
  for (int i = 5; i >= 0; i--) {
    binaryArray[i] = num & 1;
    num >>= 1;
  }
}

SensorController::BUTTON_STATE SensorController::readButton() {
  return (!digitalRead(BUTTON_PIN)) == HIGH ? SensorController::PRESSED
                                            : SensorController::UNPRESSED;
}

void SensorController::init() {
  // Initialize button pin as a pull-up input
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialize encoder pins as pull-up inputs
  pinMode(ENCODER_PIN_A1, INPUT_PULLUP); // Encoder A
  pinMode(ENCODER_PIN_A2, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B1, INPUT_PULLUP); // Encoder B
  pinMode(ENCODER_PIN_B2, INPUT_PULLUP);

  // Initialize line sensor pins as inputs
  pinMode(LINE_SENSOR_PIN_A1, INPUT); // Line Sensor A
  pinMode(LINE_SENSOR_PIN_A2, INPUT);
  pinMode(LINE_SENSOR_PIN_A3, INPUT);
  pinMode(LINE_SENSOR_PIN_B1, INPUT); // Line Sensor B
  pinMode(LINE_SENSOR_PIN_B2, INPUT);
  pinMode(LINE_SENSOR_PIN_B3, INPUT);

  // Initialize ultrasonic sensor pins
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  // Reset encoders
  zeroEncoders();

  // Initialize LED pin as an output
  pinMode(LED_PIN, OUTPUT);

  turnLED(OFF);
}

void SensorController::turnLED(LED_STATE state) {
  if (currentLEDstate != state) {
    currentLEDstate = state;
    digitalWrite(LED_PIN, state);
  } else {
    // do nothing.
  }
}

bool SensorController::buttonCheck() {
  if (readButton() == SensorController::PRESSED) {
    // Button is pressed
    turnLED(SensorController::SensorController::ON);
    return true;
  } else {
    // Button is not pressed
    turnLED(SensorController::SensorController::OFF);
    return false;
  }
}

unsigned long SensorController::getLastUltrasonicRead() {
  return lastUltrasonicRead;
}

int SensorController::getFullIRReadingResults(bool printResults) {
  int lineSensorValueA1 = lineSensorA1.cleanRead();
  int lineSensorValueA2 = lineSensorA2.cleanRead();
  int lineSensorValueA3 = lineSensorA3.cleanRead();
  int lineSensorValueB1 = lineSensorB1.cleanRead();
  int lineSensorValueB2 = lineSensorB2.cleanRead();
  int lineSensorValueB3 = lineSensorB3.cleanRead();

  // Debug messages.
  if (printResults) {
    Serial.print("Line sensors: ");
    Serial.print("[");
    Serial.print(lineSensorValueA1);
    Serial.print(", ");
    Serial.print(lineSensorValueA2);
    Serial.print(", ");
    Serial.print(lineSensorValueA3);
    Serial.print("], [");
    Serial.print(lineSensorValueB1);
    Serial.print(", ");
    Serial.print(lineSensorValueB2);
    Serial.print(", ");
    Serial.print(lineSensorValueB3);
    Serial.println("], ");
  }

  return combineLineResult(lineSensorValueA1, lineSensorValueA2, lineSensorValueA3,
                           lineSensorValueB1, lineSensorValueB2, lineSensorValueB3);
}
