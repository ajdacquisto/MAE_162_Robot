// This is the main Arduino library, which provides the core functionality for
// Arduino.
#include <Arduino.h>
// This is the configuration file for this project. It contains definitions for
// various constants and settings.
#include "config.h"
// This library provides an interface for managing the state of the system.
#include "SystemStateHandler.h"
// This library provides an interface for controlling all motors.
#include "MotorController.h"
// This library provides an interface for reading sensor values.
#include "SensorController.h"
// This library provides an interface for handling control gains.
#include "ControlGainHandler.h"
// This library provides an interface for handling branches on the track.
#include "BranchHandler.h"

// ===== GLOBAL VARIABLES =====
SystemStateHandler systemStateHandler =
    SystemStateHandler();                               // System state handler
MotorController motorController = MotorController();    // Motor controller
SensorController sensorController = SensorController(); // Sensor controller
ControlGainHandler lineSensorGainHandler =
    ControlGainHandler(LINE_FOLLOW_REGULAR_KP, LINE_FOLLOW_REGULAR_KD,
                       0.0); // Line sensor gain handler
ControlGainHandler encoderGainHandler =
    ControlGainHandler(ENCODER_DRIVE_KP, ENCODER_DRIVE_KD,
                       ENCODER_DRIVE_KI);      // Encoder gain handler
BranchHandler branchHandler = BranchHandler(); // Branch handler

// ===== ENUMS =====
enum LED_STATE { OFF = LOW, ON = HIGH };

enum BUTTON_STATE { PRESSED = HIGH, UNPRESSED = LOW };

enum componentCode {
  SERVO_A,   // 0 (right)
  SERVO_B,   // 1 (left)
  STEPPER_A, // 2 (four-bar)
  STEPPER_B, // 3 (lift)
};

enum LINE_FOLLOW_MODE { PICKUP, REGULAR, DROPOFF };

enum FOUR_BAR_DIRECTION { LOAD, UNLOAD };

// ===== FUNCTION PROTOTYPES =====
void handleTest();
void handleIdle();
void handleIRIdle();
void handleUltraSonicIdle();
void handleFollowLine(int mode);
void handleAvoidObstacle();
void initializePins();
void initializeSerialPort();
void turnLED(LED_STATE state);
BUTTON_STATE getBUTTON_STATE();
void logError(const char *message);
void handleCalibrate(int componentCode);
void handlePIDEncoderDrive(int baseSpeed);
void handleFourBar(int direction);
void resetAllPIDMemory();
void handleRotation(MotorController::ROTATE_DIRECTION direction);

// ===== MAIN SETUP =====
void setup() {
  Serial.println("Starting...");

  delay(1000);

  initializePins();
  initializeSerialPort();
  motorController.attachServoMotors();
  motorController.setStepperMotorSpeedsToMax();
  sensorController.zeroEncoders();

  systemStateHandler.changeState(SystemState::IR_IDLE);
}

// ===== MAIN LOOP =====
void loop() {
  // On state change.
  if (systemStateHandler.isNewStateFlowIndex()) {
    resetAllPIDMemory();
    // branchHandler.reset();
    delay(500);
  }

  // Order of operations
  switch (systemStateHandler.getStateFlowIndex()) {
  case 0:
    // IDLE until button press
    systemStateHandler.changeState(SystemState::IDLE);
    break;
  case 1:
    // Line follow to pickup location 1
    systemStateHandler.changeState(SystemState::LINE_FOLLOW_PICKUP);
    branchHandler.setTargetNum(0);
    break;
  case 2:
    // Rotate to face pickup location 1
    if (motorController.getDirectionToRotate(PICKUP_LOCATION_1) ==
        MotorController::LEFT) {
      systemStateHandler.changeState(SystemState::ROTATE_LEFT);
    } else {
      systemStateHandler.changeState(SystemState::ROTATE_RIGHT);
    }
    break;
  case 3:
  case 8:
    // Use ultrasonic sensor to approach the pickup location
    systemStateHandler.changeState(SystemState::ULTRASONIC_APPROACH);
    break;
  case 4:
  case 9:
    // Use ultrasonic sensor and encoder drive to back straight up
    systemStateHandler.changeState(SystemState::ULTRASONIC_REVERSE);
    break;
  case 5:
    // Rotate to face front again.
    if (motorController.getDirectionToRotate(PICKUP_LOCATION_1) ==
        MotorController::LEFT) {
      systemStateHandler.changeState(SystemState::ROTATE_RIGHT);
    } else {
      systemStateHandler.changeState(SystemState::ROTATE_LEFT);
    }
    break;
  case 6:
    // Line follow to pickup location 2
    systemStateHandler.changeState(SystemState::LINE_FOLLOW_PICKUP);
    branchHandler.setTargetNum(1);
    break;
  case 7:
    // Rotate to face pickup location 2
    if (motorController.getDirectionToRotate(PICKUP_LOCATION_2) ==
        MotorController::LEFT) {
      systemStateHandler.changeState(SystemState::ROTATE_LEFT);
    } else {
      systemStateHandler.changeState(SystemState::ROTATE_RIGHT);
    }
    break;
  case 10:
    // Rotate to face front again.
    if (motorController.getDirectionToRotate(PICKUP_LOCATION_2) ==
        MotorController::LEFT) {
      systemStateHandler.changeState(SystemState::ROTATE_RIGHT);
    } else {
      systemStateHandler.changeState(SystemState::ROTATE_LEFT);
    }
    break;
  }

  switch (systemStateHandler.getCurrentState()) {
  case SystemState::TEST:
    // Code for testing
    handleTest();
    break;
  case SystemState::CALIBRATE:
    // Code for calibrating stepper positions
    handleCalibrate(SERVO_A);
    break;
  case SystemState::IDLE:
    // Code for simple idle state
    handleIdle();
    break;
  case SystemState::IR_IDLE:
    // Code for idle state with IR sensors active
    handleIRIdle();
    break;
  case SystemState::ULTRASONIC_IDLE:
    // Code for idle state with ultrasonic sensor active
    break;
  case SystemState::PID_ENCODER_DRIVE:
    // Code for PID control of encoder drive
    handlePIDEncoderDrive(255);
    break;
  case SystemState::FOLLOW_LINE:
    // Code for simple line following
    handleFollowLine(REGULAR);
    break;
  case SystemState::LINE_FOLLOW_PICKUP:
    handleFollowLine(PICKUP);
    break;
  case SystemState::LINE_FOLLOW_DROPOFF:
    handleFollowLine(DROPOFF);
    break;
  case SystemState::FOUR_BAR_LOAD:
    // Code for four-bar mechanism
    handleFourBar(LOAD);
    break;
  case SystemState::FOUR_BAR_UNLOAD:
    // Code for four-bar mechanism
    handleFourBar(UNLOAD);
    break;
  case SystemState::ROTATE_LEFT:
    handleRotation(MotorController::LEFT);
    break;
  case SystemState::ROTATE_RIGHT:
    handleRotation(MotorController::RIGHT);
    break;
  case SystemState::ULTRASONIC_APPROACH:
    handleUltrasonicApproach();
    break;
  default:
    logError("Invalid state");
    break;
  }
}

// ===== OTHER FUNCTIONS =====
void handleTest() {
  // Code for testing

  const int DELAY_BETWEEN_LOOPS = 5000; // 5 seconds

  const int MOTOR_A_FWD_TEST_START = 0;
  const int MOTOR_A_TEST_LENGTH = 1000; // 1 second

  const int MOTOR_A_REV_TEST_START =
      MOTOR_A_FWD_TEST_START + MOTOR_A_TEST_LENGTH;

  const int MOTOR_B_FWD_TEST_START =
      MOTOR_A_REV_TEST_START + MOTOR_A_TEST_LENGTH;
  const int MOTOR_B_TEST_LENGTH = 1000; // 1 second

  const int MOTOR_B_REV_TEST_START =
      MOTOR_B_FWD_TEST_START + MOTOR_B_TEST_LENGTH;

  const int STEPPER_A_FWD_TEST_START =
      MOTOR_B_REV_TEST_START + MOTOR_B_TEST_LENGTH;
  const int STEPPER_A_TEST_LENGTH = 5000; // 5 seconds

  const int STEPPER_A_REV_TEST_START =
      STEPPER_A_FWD_TEST_START + STEPPER_A_TEST_LENGTH;

  const int STEPPER_B_FWD_TEST_START =
      STEPPER_A_REV_TEST_START + STEPPER_A_TEST_LENGTH;
  const int STEPPER_B_TEST_LENGTH = 5000; // 5 seconds

  const int STEPPER_B_REV_TEST_START =
      STEPPER_B_FWD_TEST_START + STEPPER_B_TEST_LENGTH;

  static unsigned long myTimerStart = millis();
  Serial.print("Time: ");
  Serial.print(millis() - myTimerStart);
  Serial.print(", ");

  if (getBUTTON_STATE() == PRESSED) {
    turnLED(ON);
    logError("Button pressed");
  } else {
    turnLED(OFF);

    if (millis() - myTimerStart <
        MOTOR_A_FWD_TEST_START + MOTOR_A_TEST_LENGTH) {
      Serial.print("Motor A Forward");
      // Test motor A (right) FORWARD
      motorController.motorDriver.motorAForward();
      motorController.motorDriver.motorBStop();
    } else if (millis() - myTimerStart <
               MOTOR_A_REV_TEST_START + MOTOR_A_TEST_LENGTH) {
      Serial.print("Motor A Reverse");
      // Test motor A (right) REVERSE
      motorController.motorDriver.motorAReverse();
      motorController.motorDriver.motorBStop();
    } else if (millis() - myTimerStart <
               MOTOR_B_FWD_TEST_START + MOTOR_B_TEST_LENGTH) {
      Serial.print("Motor B Forward");
      // Test motor B (left) FORWARD
      motorController.motorDriver.motorBForward();
      motorController.motorDriver.motorAStop();
    } else if (millis() - myTimerStart <
               MOTOR_B_REV_TEST_START + MOTOR_B_TEST_LENGTH) {
      Serial.print("Motor B Reverse");
      // Test motor B (left) REVERSE
      motorController.motorDriver.motorBReverse();
      motorController.motorDriver.motorAStop();
    } else if (millis() - myTimerStart <
               STEPPER_A_FWD_TEST_START + STEPPER_A_TEST_LENGTH) {
      Serial.print("Stepper A Forward");
      // Test stepper A (four-bar) FORWARD
      motorController.servosOff();
      motorController.rotateStepperAsteps(1);
    } else if (millis() - myTimerStart <
               STEPPER_A_REV_TEST_START + STEPPER_A_TEST_LENGTH) {
      Serial.print("Stepper A Reverse");
      // Test stepper A (four-bar) REVERSE
      motorController.servosOff();
      motorController.rotateStepperAsteps(-1);
    } else if (millis() - myTimerStart <
               STEPPER_B_FWD_TEST_START + STEPPER_B_TEST_LENGTH) {
      Serial.print("Stepper B Forward");
      // Test stepper B (lift) FORWARD
      motorController.servosOff();
      motorController.rotateStepperBsteps(1);
    } else if (millis() - myTimerStart <
               STEPPER_B_REV_TEST_START + STEPPER_B_TEST_LENGTH) {
      Serial.print("Stepper B Reverse");
      // Test stepper B (lift) REVERSE
      motorController.servosOff();
      motorController.rotateStepperBsteps(-1);
    } else {
      Serial.print("Resetting...");
      delay(DELAY_BETWEEN_LOOPS);
      myTimerStart = millis();
    }
  }
  Serial.println();
}

void handleIdle() {
  motorController.servosOff();
  turnLED(ON);
  if (getBUTTON_STATE() == PRESSED) {
    turnLED(OFF);
    systemStateHandler.advanceStateFlowIndex();
  }
}

void handleCalibrate(int componentCode) {
  switch (componentCode) {
  case STEPPER_A:
    if (getBUTTON_STATE() == PRESSED) {
      // Hold down button until four-bar crank is in lowest position.
      turnLED(ON);
      motorController.rotateStepperAsteps(1);
      delay(200);
    } else {
      turnLED(OFF);
    }
    break;
  case STEPPER_B:
    if (getBUTTON_STATE() == PRESSED) {
      // Hold down button until lift is in lowest position.
      turnLED(ON);
      motorController.rotateStepperBsteps(1);
      delay(200);
    } else {
      turnLED(OFF);
    }
    break;
  case SERVO_A:
    if (getBUTTON_STATE() == PRESSED) {
      motorController.motorDriver.motorAForward(64); // 25% speed
      turnLED(ON);
    } else {
      motorController.servosOff();
      turnLED(OFF);
    }
    break;
  case SERVO_B:
    if (getBUTTON_STATE() == PRESSED) {
      motorController.motorDriver.motorBForward(64); // 25% speed
      turnLED(ON);
    } else {
      motorController.servosOff();
      turnLED(OFF);
    }
    break;
  default:
    logError("Invalid component code");
    break;
  }
}

void handleIRIdle() {
  // Read the sensor values
  sensorController.readLineSensorA();
  sensorController.readLineSensorB();

  int resultsA = sensorController.getLineResultA();
  int resultsB = sensorController.getLineResultB();

  Serial.print("Combined Line Sensor Value: ");
  Serial.print(resultsA, BIN);
  Serial.print(", ");
  Serial.println(resultsB, BIN);

  motorController.servosOff();
}

void handleUltraSonicIdle() {
  // Read the ultrasonic sensor
  long distance = sensorController.getUltrasonicDistance();

  // Print the distance
  Serial.print("Distance: ");
  Serial.println(distance);

  motorController.servosOff();

  delay(100);
}

void handlePIDEncoderDrive(int baseSpeed) {
  // PID parameters
  float Kp = encoderGainHandler.getKp();
  float Ki = encoderGainHandler.getKi();
  float Kd = encoderGainHandler.getKd();

  // Read encoder values
  long encoderValueA = sensorController.readEncoderA();
  long encoderValueB = sensorController.readEncoderB();

  // Calculate error
  long error = encoderValueA - encoderValueB;

  // PID control
  encoderGainHandler.incrementIntegral(error);
  float derivative = error - encoderGainHandler.getLastError();
  float output =
      Kp * error + Ki * encoderGainHandler.getIntegral() + Kd * derivative;

  // Adjust motor speeds
  int motorSpeedA = constrain(baseSpeed - output, 0, 255);
  int motorSpeedB = constrain(baseSpeed + output, 0, 255);

  motorController.motorDriver.motorAForward(motorSpeedA);
  motorController.motorDriver.motorBForward(motorSpeedB);

  // Debugging output
  Serial.print("Left: ");
  Serial.print(encoderValueA);
  Serial.print(" Right: ");
  Serial.print(encoderValueB);
  Serial.print(" Error: ");
  Serial.println(error);

  // Update previous error
  encoderGainHandler.setLastError(error);

  // Short delay to avoid overwhelming the microcontroller
  delay(100);
}

void handleFollowLine(int mode) {
  // Turn sensor B on for pickup and dropoff modes only.
  bool isSensorBOn = (mode == PICKUP || mode == DROPOFF);

  // Read the sensor values
  sensorController.readLineSensorA();
  if (isSensorBOn) {
    sensorController.readLineSensorB();
  }

  // Calculate the line result
  int resultA = sensorController.getLineResultA();

  int resultB = 0;
  if (isSensorBOn) {
    resultB = sensorController.getLineResultB();
  }

  Serial.print("Line sensors: ");
  Serial.print(resultA, BIN);
  Serial.print(", ");
  Serial.println(resultB, BIN);

  switch (mode) {
  case PICKUP: {
    // Code for line following in pickup mode

    int targetLocation = 0;
    if (branchHandler.getTargetNum() == 0) {
      targetLocation = PICKUP_LOCATION_1;
    } else if (branchHandler.getTargetNum() == 1) {
      targetLocation = PICKUP_LOCATION_2;
    } else {
      logError("Invalid target number");
    }

    int targetBranchNum =
        branchHandler.getTargetBranchNumFromLocation(targetLocation);

    branchHandler.doBranchCheck(resultB);
    bool atBranch = branchHandler.getIsCurrentlyOverBranch();
    int branchNum = branchHandler.getCurrentLocation();

    if (atBranch && (branchNum == targetBranchNum)) {
      // STOP.
      systemStateHandler.advanceStateFlowIndex();
      return;
    } else {
      // Follow the line.
    }

    break;
  }
  case REGULAR: {
    // Code for regular line following
    int error = sensorController.determineError(resultA);

    int P = error;
    int D = error - lineSensorGainHandler.getLastError();
    float Kp = lineSensorGainHandler.getKp();
    float Kd = lineSensorGainHandler.getKd();
    int output = Kp * P + Kd * D;

    int baseSpeed = 150; // Adjust this value as needed
    int rightMotorSpeed = baseSpeed - output;
    int leftMotorSpeed = baseSpeed + output;

    // Ensure motor speeds are within valid range (e.g., 0 to 255 for PWM
    // control)
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

    if (rightMotorSpeed == leftMotorSpeed) {
      handlePIDEncoderDrive(rightMotorSpeed);
    } else {
      encoderGainHandler.reset();
      motorController.motorDriver.motorAForward(rightMotorSpeed);
      motorController.motorDriver.motorBForward(leftMotorSpeed);
    }

    lineSensorGainHandler.setLastError(error);
    break;
  }
  case DROPOFF: {
    // Code for line following in dropoff mode
    break;
  }
  default:
    logError("Invalid mode");
    break;
  }
}

void handleFourBar(int direction) {
  if (direction == LOAD) {
    // Load the four-bar mechanism
    motorController.rotateStepperAdeg(360);
  } else if (direction == UNLOAD) {
    // Unload the four-bar mechanism
    motorController.rotateStepperAdeg(-360);
  } else {
    logError("Invalid direction");
  }
}

void handleRotation(MotorController::ROTATE_DIRECTION direction) {
  if (motorController.rotateRobot(direction,
                                  sensorController.getLineResultA())) {
    systemStateHandler.advanceStateFlowIndex();
  }
}

void handleUltrasonicApproach() {
  // Read the ultrasonic sensor
  int DISTANCE_THRESHOLD = 10;
  long distance = sensorController.getUltrasonicDistance();

  if (sensorController.getUltrasonicMemory() == 0) {
    sensorController.setUltrasonicMemory(distance);
  }

  // Print the distance
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > DISTANCE_THRESHOLD) {
    handleFollowLine(REGULAR);
  } else {
    motorController.servosOff();
    systemStateHandler.advanceStateFlowIndex();
  }

  delay(100);
}

void handleUltrasonicReverse() {
  int REVERSE_SPEED = 64;

  long distance = sensorController.getUltrasonicDistance();

  // Print the distance
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < sensorController.getUltrasonicMemory()) {
    handlePIDEncoderDrive(-REVERSE_SPEED);
  } else {
    motorController.servosOff();
    systemStateHandler.advanceStateFlowIndex();
  }
}

// ===== HELPER FUNCTIONS =====
void initializePins() {
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

  // Initialize LED pin as an output
  pinMode(LED_PIN, OUTPUT);
}

void initializeSerialPort() {
  Serial.begin(9600); // Initialize Serial port
  while (!Serial)
    ; // Waits for the Serial port to connect.
}

void turnLED(LED_STATE state) { digitalWrite(LED_PIN, state); }

BUTTON_STATE getBUTTON_STATE() {
  return (!digitalRead(BUTTON_PIN)) == HIGH ? PRESSED : UNPRESSED;
}

void logError(const char *message) {
  systemStateHandler.changeState(SystemState::IDLE);
  turnLED(ON);
  motorController.servosOff();
  Serial.println(message);
  while (true)
    ; // Stop the program
}

void resetAllPIDMemory() {
  lineSensorGainHandler.reset();
  encoderGainHandler.reset();
}
