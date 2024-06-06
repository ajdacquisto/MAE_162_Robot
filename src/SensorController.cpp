#include "SensorController.h"

// Constructor
SensorController::SensorController()
    : newIR(NEW_IR_PIN_1, NEW_IR_PIN_2, NEW_IR_PIN_3, NEW_IR_PIN_4,
            NEW_IR_PIN_5),
      encFrontRight(ENCODER_PIN_C1, ENCODER_PIN_C2),
      encFrontLeft(ENCODER_PIN_D1, ENCODER_PIN_D2),
      encRearRight(ENCODER_PIN_A1, ENCODER_PIN_A2),
      encRearLeft(ENCODER_PIN_B1, ENCODER_PIN_B2),
      lineSensorA1(LINE_SENSOR_PIN_A1),         // Line sensor A1
      lineSensorA2(LINE_SENSOR_PIN_A2),         // Line sensor A2
      lineSensorA3(LINE_SENSOR_PIN_A3),         // Line sensor A3
      lineSensorB1(LINE_SENSOR_PIN_B1),         // Line sensor B1
      lineSensorB2(LINE_SENSOR_PIN_B2),         // Line sensor B2
      lineSensorB3(LINE_SENSOR_PIN_B3)          // Line sensor B3
{
  lineSensorAThreshold = LINE_SENSOR_THRESHOLD;
}

// Destructor
SensorController::~SensorController() {
  // Clean up any resources
}

// ===== SENSOR READINGS =====

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

int SensorController::combineLineResult(int avg1, int avg2, int avg3, int avg4,
                                        int avg5, int avg6) {
  // Define the average values for each sensor
  const int AVERAGE_VALUES[6] = {770, 712, 796, 787, 765, 847};

  // Calculate the threshold for each sensor dynamically based on the provided
  // averages
  int lineSensorValueA1 =
      (avg1 > (AVERAGE_VALUES[0] + 10 + 0 * (1000 - AVERAGE_VALUES[0]) / 2))
          ? 1
          : 0;
  int lineSensorValueA2 =
      (avg2 > (AVERAGE_VALUES[1] + 25 + 0 * (1000 - AVERAGE_VALUES[1]) / 2))
          ? 1
          : 0;
  int lineSensorValueA3 =
      (avg3 > (AVERAGE_VALUES[2] - 0 + 0 * (1000 - AVERAGE_VALUES[2]) / 2)) ? 1
                                                                            : 0;
  int lineSensorValueB1 =
      (avg4 > (AVERAGE_VALUES[3] - 0 + 0 * (1000 - AVERAGE_VALUES[3]) / 2)) ? 1
                                                                            : 0;
  int lineSensorValueB2 =
      (avg5 > (AVERAGE_VALUES[4] - 0 + 0 * (1000 - AVERAGE_VALUES[4]) / 2)) ? 1
                                                                            : 0;
  int lineSensorValueB3 =
      (avg6 > (AVERAGE_VALUES[5] - 0 + 0 * (1000 - AVERAGE_VALUES[5]) / 2)) ? 1
                                                                            : 0;

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

int SensorController::getLineSensorAThreshold() { return lineSensorAThreshold; }

int SensorController::getLineSensorBThreshold() { return lineSensorBThreshold; }

void SensorController::setLineSensorAThreshold(int threshold) {
  lineSensorAThreshold = threshold;
}

void SensorController::setLineSensorBThreshold(int threshold) {
  lineSensorBThreshold = threshold;
}

// ===== ENCODER SPEED STUFF =====

float SensorController::getEncFrontRightSpeed() {
  unsigned long currentTime = millis();
  long encoderReading = encFrontRight.read();
  float deltaTime = (currentTime - encFrontRightLastTime) / 1000.0;
  long deltaDistance = encoderReading - encFrontRightLastVal;
  encFrontRightLastTime = currentTime;
  encFrontRightLastVal = encoderReading;
  return deltaDistance / deltaTime;
}

float SensorController::getEncFrontLeftSpeed() {
  unsigned long currentTime = millis();
  long encoderReading = encFrontLeft.read();
  float deltaTime = (currentTime - encFrontLeftLastTime) / 1000.0;
  long deltaDistance = encoderReading - encFrontLeftLastVal;
  encFrontLeftLastTime = currentTime;
  encFrontLeftLastVal = encoderReading;
  return deltaDistance / deltaTime;
}

float SensorController::getEncRearRightSpeed() {
  unsigned long currentTime = millis();
  long encoderReading = encRearRight.read();
  float deltaTime = (currentTime - encRearRightLastTime) / 1000.0;
  long deltaDistance = encoderReading - encRearRightLastVal;
  encRearRightLastTime = currentTime;
  encRearRightLastVal = encoderReading;
  return deltaDistance / deltaTime;
}

float SensorController::getEncRearLeftSpeed() {
  unsigned long currentTime = millis();
  long encoderReading = encRearLeft.read();
  float deltaTime = (currentTime - encRearLeftLastTime) / 1000.0;
  long deltaDistance = encoderReading - encRearLeftLastVal;
  encRearLeftLastTime = currentTime;
  encRearLeftLastVal = encoderReading;
  return deltaDistance / deltaTime;
}

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

  newIR.init();

  // Initialize ultrasonic sensor pins
  ultrasonicHandler.init();

  // Reset encoders
  zeroAllEncoders();

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

int SensorController::getFullIRReadingResults(bool printResults, bool isNewIR) {
  if (!isNewIR) {
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

    return combineLineResult(lineSensorValueA1, lineSensorValueA2,
                             lineSensorValueA3, lineSensorValueB1,
                             lineSensorValueB2, lineSensorValueB3);
  } else {
    return newIR.readSensorData();
  }
}

NewIRSensor SensorController::getNewIRSensor() { return newIR; }

UltrasonicHandler SensorController::getUltrasonicHandler() {
  return ultrasonicHandler;
}

void SensorController::zeroEncoder(Encoder &encoder) { encoder.write(0); }

void SensorController::zeroAllEncoders() {
  // Zero all encoders
  zeroEncoder(encFrontRight);
  zeroEncoder(encFrontLeft);
  zeroEncoder(encRearRight);
  zeroEncoder(encRearLeft);
}

long SensorController::readEncoder(Encoder &encoder) { return encoder.read(); }