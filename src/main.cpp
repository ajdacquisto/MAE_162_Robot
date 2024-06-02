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
// This library provides an interface for handling serial communication.
#include "SerialController.h"
// This library provides an interface for handling look-ahead line following.
#include "LookAhead.h"

// ===== GLOBAL VARIABLES =====
SystemState::State DEFAULT_STATE = SystemState::IDLE;

MotorController::COMPONENT CALIBRATE_COMPONENT = MotorController::LEFT_WHEEL;
MotorController::MOTOR_DIRECTION CALIBRATE_DIRECTION = MotorController::FORWARD;

// ===== CONTROL OBJECTS =====
SystemStateHandler systemStateHandler =
    SystemStateHandler();                               // System state handler
MotorController motorController = MotorController();    // Motor controller
SensorController sensorController = SensorController(); // Sensor controller
ControlGainHandler lineSensorGainHandler =
    ControlGainHandler(LINE_FOLLOW_REGULAR_KP, LINE_FOLLOW_REGULAR_KI,
                       LINE_FOLLOW_REGULAR_KD); // Line sensor gain handler
ControlGainHandler encoderGainHandler =
    ControlGainHandler(ENCODER_DRIVE_KP, ENCODER_DRIVE_KI,
                       ENCODER_DRIVE_KD);      // Encoder gain handler
BranchHandler branchHandler = BranchHandler(); // Branch handler
SerialController serialController = SerialController();
LookAhead lookAhead = LookAhead();

// ===== ENUMS =====
enum LINE_FOLLOW_MODE { PICKUP, REGULAR, DROPOFF };
enum ROTATE_TYPE { TOWARDS, AWAY_FROM };

// ===== FUNCTION PROTOTYPES =====

// State handlers
void handleTest();
void handleIdle();
void handleIRIdle();
void handleUltraSonicIdle();
void handleFollowLine(int mode);
void handleCalibrate(MotorController::COMPONENT componentCode);
void handlePIDEncoderDrive(int BASE_SPEED);
void handleRotation(MotorController::ROTATE_DIRECTION direction);
void handleUltrasonicApproach();
void handleUltrasonicReverse();
void handleLift(int direction);
void handleLookAheadLineFollow();

// Helper functions
void resetAllPIDMemory();
void calculateRotation(int rotationType, int targetLocation);

// LOGGING
void logError(const char *message);

// ===== MAIN SETUP =====
void setup() {
  delay(1000);
  serialController.init();
  sensorController.init();
  motorController.init();
  systemStateHandler.init(DEFAULT_STATE);
}

// ===== MAIN LOOP =====
void loop() {
  if (systemStateHandler.isNewStateFlowIndex()) {
    serialController.printlnWithTimestamp("STATE CHANGE");
    // On state change.
    resetAllPIDMemory();
    motorController.servosOff();
    motorController.setStepperMotorSpeedsToMax();
    // branchHandler.reset();
    delay(500);
  }

  SystemState::State currentState = systemStateHandler.getCurrentState();

  // Order of operations
  switch (systemStateHandler.getStateFlowIndex()) {
  case 0:
    // IDLE until button press
    systemStateHandler.changeState(DEFAULT_STATE);
    break;
  case 1:
    // Line follow to pickup location 1
    if (currentState != SystemState::LINE_FOLLOW_PICKUP) {
      serialController.printlnWithTimestamp("changeState call.");
      systemStateHandler.changeState(SystemState::LINE_FOLLOW_PICKUP);
      branchHandler.setTargetNum(1);
    }
    break;
  case 2:
    // Rotate to face pickup location 1
    calculateRotation(TOWARDS, PICKUP_LOCATION_1);
    break;
  case 3:
    // Use ultrasonic sensor to approach the pickup location
    systemStateHandler.changeState(SystemState::ULTRASONIC_APPROACH);
    break;
  case 4:
    // Load using the four-bar mechanism
    systemStateHandler.changeState(SystemState::FOUR_BAR_LOAD);
    break;
  case 5:
    // Lower the lift.
    systemStateHandler.changeState(SystemState::LIFT_LOWER);
    break;
  case 6:
    // Use ultrasonic sensor and encoder drive to back straight up
    systemStateHandler.changeState(SystemState::ULTRASONIC_REVERSE);
    break;
  case 7:
    // Rotate to face front again.
    calculateRotation(AWAY_FROM, PICKUP_LOCATION_1);
    break;
  case 8:
    // Line follow to pickup location 2
    systemStateHandler.changeState(SystemState::LINE_FOLLOW_PICKUP);
    branchHandler.setTargetNum(2);
    break;
  case 9:
    // Rotate to face pickup location 2
    calculateRotation(TOWARDS, PICKUP_LOCATION_2);
    break;
  case 10:
    // [SAME AS STEP 3]
    systemStateHandler.changeState(SystemState::ULTRASONIC_APPROACH);
    break;
  case 11:
    // [SAME AS STEP 4]
    systemStateHandler.changeState(SystemState::FOUR_BAR_LOAD);
    break;
  case 12:
    // [SAME AS STEP 5]
    systemStateHandler.changeState(SystemState::LIFT_LOWER);
    break;
  case 13:
    // [SAME AS STEP 6]
    systemStateHandler.changeState(SystemState::ULTRASONIC_REVERSE);
    break;
  case 14:
    // Rotate to face front again.
    calculateRotation(AWAY_FROM, PICKUP_LOCATION_2);
    break;
  case 15:
    // Line follow until obstacle.
    break;
  case 16:
    // Line follow in dropoff mode.
    break;
  case 17:
    // Rotate to face dropoff location.
    break;
  case 18:
    // Use ultrasonic sensor to approach the dropoff location.
    break;
  case 19:
    // Raise the lift
    break;
  case 20:
    // Unload using the four-bar mechanism
    break;
  case 21:
    // Rotate to face front again.
    break;
  case 22:
    // Line follow to end of track.
    break;
  case 23:
    // IDLE
    systemStateHandler.changeState(SystemState::IDLE);
    break;
  default:
    // Error handling
    logError("Invalid state flow index");
    break;
  }

  switch (systemStateHandler.getCurrentState()) {
  case SystemState::TEST:
    // Code for testing
    handleTest();
    break;
  case SystemState::CALIBRATE:
    // Code for calibrating stepper positions
    handleCalibrate(CALIBRATE_COMPONENT);
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
    handleUltraSonicIdle();
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
    // Code for line following in pickup mode
    sensorController.buttonCheck();
    handleFollowLine(PICKUP);
    break;
  case SystemState::LINE_FOLLOW_DROPOFF:
    // Code for line following in dropoff mode
    handleFollowLine(DROPOFF);
    break;
  case SystemState::FOUR_BAR_LOAD:
    // Code for four-bar mechanism
    motorController.handleFourBar(MotorController::LOAD);
    break;
  case SystemState::FOUR_BAR_UNLOAD:
    // Code for four-bar mechanism
    motorController.handleFourBar(MotorController::UNLOAD);
    break;
  case SystemState::ROTATE_LEFT:
    // Code for rotating the robot left
    handleRotation(MotorController::LEFT);
    break;
  case SystemState::ROTATE_RIGHT:
    // Code for rotating the robot right
    handleRotation(MotorController::RIGHT);
    break;
  case SystemState::ULTRASONIC_APPROACH:
    // Code for approaching an obstacle using ultrasonic sensor
    handleUltrasonicApproach();
    break;
  case SystemState::LIFT_LOWER:
    // Lower the lift.
    handleLift(MotorController::DOWN);
    break;
  case SystemState::LIFT_RAISE:
    // Raise the lift.
    handleLift(MotorController::UP);
    break;
  default:
    // Error handling
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

  if (sensorController.readButton() == SensorController::PRESSED) {
    sensorController.turnLED(SensorController::ON);
    logError("Button pressed");
  } else {
    sensorController.turnLED(SensorController::OFF);

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
  sensorController.turnLED(SensorController::ON);
  if (sensorController.readButton() == SensorController::PRESSED) {
    sensorController.turnLED(SensorController::OFF);
    serialController.printlnWithTimestamp("Exiting idle phase...");
    while (sensorController.readButton() == SensorController::PRESSED)
      ;
    systemStateHandler.advanceStateFlowIndex();
  }
}

void handleCalibrate(MotorController::COMPONENT componentCode) {
  switch (componentCode) {
  case MotorController::FOUR_BAR: {
    int FOUR_BAR_CALIBRATION_MODE = 2;

    if (FOUR_BAR_CALIBRATION_MODE == 1) {
      if (sensorController.readButton() == SensorController::PRESSED) {
        // Hold down button until four-bar crank is in lowest position.
        sensorController.turnLED(SensorController::ON);
        // motorController.rotateStepperAsteps(1);
        delay(200);
      } else {
        sensorController.turnLED(SensorController::OFF);
      }
    } else if (FOUR_BAR_CALIBRATION_MODE == 2) {
      // Just do one rotation.
      delay(1000);
      motorController.rotateStepperAdeg(360);
      while (true)
        ;
    }
    break;
  }
  case MotorController::LIFT: {
    if (sensorController.readButton() == SensorController::PRESSED) {
      // Hold down button until lift is in lowest position.
      sensorController.turnLED(SensorController::ON);
      motorController.rotateStepperBsteps(20);
      delay(200);
    } else {
      sensorController.turnLED(SensorController::OFF);
    }
    break;
  }
  case MotorController::RIGHT_WHEEL: {
    if (sensorController.readButton() == SensorController::PRESSED) {
      motorController.motorDriver.motorAForward(255); // full speed
      int actualRightSpeed = sensorController.getEncoderASpeed();
      Serial.print("Actual right speed: ");
      Serial.println(actualRightSpeed);
      sensorController.turnLED(SensorController::ON);
    } else {
      motorController.servosOff();
      sensorController.turnLED(SensorController::OFF);
    }
    break;
  }
  case MotorController::LEFT_WHEEL: {
    if (sensorController.readButton() == SensorController::PRESSED) {
      motorController.motorDriver.motorBForward(255); // full speed
      int actualLeftSpeed = sensorController.getEncoderBSpeed();
      Serial.print("Actual left speed: ");
      Serial.println(actualLeftSpeed);
      sensorController.turnLED(SensorController::ON);
    } else {
      motorController.servosOff();
      sensorController.turnLED(SensorController::OFF);
    }
    break;
  }
  case MotorController::BOTH_WHEELS: {
    delay(1000);
    int speed = 255;
    if (CALIBRATE_DIRECTION == MotorController::FORWARD) {
      motorController.servoDrive(MotorController::SERVO_A, speed);
      motorController.servoDrive(MotorController::SERVO_B, speed);
    } else {
      motorController.servoDrive(MotorController::SERVO_A, -speed);
      motorController.servoDrive(MotorController::SERVO_B, -speed);
    }
    while (sensorController.readButton() == SensorController::PRESSED) {
      motorController.servosOff();
      while (true)
        ;
    }
    break;
  }
  default:
    logError("Invalid component code");
    break;
  }
}

void handleIRIdle() {
  bool SHOW_RAW_IR_READINGS = true;
  bool SHOW_BINARY_READING = true;

  int lineSensorResults =
      sensorController.getFullIRReadingResults(SHOW_RAW_IR_READINGS);

  if (SHOW_BINARY_READING) {
    serialController.printWithTimestamp("Sensor Mapping: { ");
    serialController.printBinaryWithLeadingZeros(lineSensorResults);
    Serial.println(" }\n");
  }

  motorController.servosOff();
  delay(100);
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

void handlePIDEncoderDrive(int BASE_SPEED) {
  // PID parameters
  float Kp = encoderGainHandler.getKp();
  float Ki = encoderGainHandler.getKi();
  float Kd = encoderGainHandler.getKd();

  // Debug messages.
  Serial.print("<debug_PID> GAINS: [Kp:");
  Serial.print(Kp);
  Serial.print(", Ki:");
  Serial.print(Ki);
  Serial.print(", Kd:");
  Serial.print(Kd);
  Serial.println("]");

  // Read encoder values
  long encoderValueA = sensorController.readEncoderA();
  long encoderValueB = sensorController.readEncoderB();

  // Calculate error
  long error = encoderValueA - encoderValueB;

  // Update integral term.
  encoderGainHandler.incrementIntegral(error);

  // Calculate derivative term
  float derivative = error - encoderGainHandler.getLastError();

  // PID control
  float output =
      Kp * error + Ki * encoderGainHandler.getIntegral() + Kd * derivative;

  // Update previous error.
  encoderGainHandler.setLastError(error);

  // Adjust motor speeds
  int motorSpeedA = constrain(BASE_SPEED - output, 0, 255);
  int motorSpeedB = constrain(BASE_SPEED + output, 0, 255);

  // Enable motors.
  motorController.servoDrive(MotorController::SERVO_A, motorSpeedA);
  motorController.servoDrive(MotorController::SERVO_B, motorSpeedB);

  // Debugging output
  Serial.print("<debug_PID> Encoder values - Left: ");
  Serial.print(encoderValueA);
  Serial.print(", Right: ");
  Serial.print(encoderValueB);
  Serial.print(", Error: ");
  Serial.println(error);

  Serial.print("<debug_PID> EncoderDrive speeds - servoA: ");
  Serial.print(motorSpeedA);
  Serial.print(", servoB: ");
  Serial.println(motorSpeedB);

  // Short delay to avoid overwhelming the microcontroller
  // delay(100);
}

void handleFollowLine(int mode) {
  bool DO_LOOK_AHEAD = true;

  if (DO_LOOK_AHEAD) {
    handleLookAheadLineFollow();
    return;
  }

  bool DO_ULTRASONIC_CHECK_OBSTACLE = false;
  bool DO_ULTRASONIC_CHECK_TURN = false;
  bool SHOW_RAW_IR_READINGS = true;
  bool PRINT_DESIRED_SPEEDS = true;
  bool PRINT_ACTUAL_SPEEDS = true;
  bool PRINT_MOTOR_COMMAND = true;
  bool DO_EMERGENCY_REVERSE = true;

  serialController.printlnWithTimestamp("Start of handleFollowLine.");
  // Read the sensor values

  bool SHOW_BINARY_READING = true;

  int lineSensorResults =
      sensorController.getFullIRReadingResults(SHOW_RAW_IR_READINGS);

  if (SHOW_BINARY_READING) {
    serialController.printWithTimestamp("Sensor Mapping: { ");
    serialController.printBinaryWithLeadingZeros(lineSensorResults);
    Serial.println(" }\n");
  }

  switch (mode) {
  case PICKUP: { // Code for line following in pickup mode
    // Check if at target branch.
    if ((millis() - systemStateHandler.getLastStateChangeTime()) > 1000 &&
        branchHandler.isAtTargetLocation()) {
      // STOP.
      Serial.println("<debug> At branch, stopping.");
      systemStateHandler.advanceStateFlowIndex();
      return;
    } else {
      // Follow the line.
    }
    break;
  }
  case REGULAR: {
    break;
  }
  case DROPOFF: {
    break;
  }
  default:
    logError("Invalid mode");
    break;
  }

  if (millis() - lineSensorGainHandler.getLastIntegralResetTime() > 5000) {
    lineSensorGainHandler.resetIntegral();
  }

  // PID Line stuff...
  int lineError = -sensorController.determineError(lineSensorResults);

  if (lineError == 0) {
    lineSensorGainHandler.resetIntegral();
  }

  serialController.printWithTimestamp("Line error: ");
  Serial.println(lineError);

  // ============================
  // ===== ULTRASONIC CHECK =====s
  if (DO_ULTRASONIC_CHECK_OBSTACLE) {
    if (sensorController.isObstacle(OBSTACLE_DISTANCE_THRESHOLD)) {
      serialController.printlnWithTimestamp("<!> Obstacle detected.");
      motorController.servosOff();
      delay(100);
      return;
    }
  } else if (DO_ULTRASONIC_CHECK_TURN) {
    if (sensorController.isObstacle(NINETY_DEGREE_TURN_DISTANCE)) {
      serialController.printlnWithTimestamp("<!> 90 degree turn detected.");
      motorController.servosOff();
      while (true)
        ;
      delay(100);
      return;
    }
  }
  // ============================
  // ============================

  // =====================================
  // ===== EMERGENCY REVERSE COMMAND =====
  if (DO_EMERGENCY_REVERSE && lineError == 99) {
    serialController.printlnWithTimestamp("<!> Reverse command");
    motorController.servosOff();
    motorController.servoDrive(MotorController::SERVO_A, -REVERSE_SPEED);
    motorController.servoDrive(MotorController::SERVO_B, -REVERSE_SPEED);
    delay(100);
    return;
  } else if (!DO_EMERGENCY_REVERSE) {
    lineError = lineSensorGainHandler.getLastError();
  }
  // =====================================
  // =====================================

  // PID Line stuff cont'd...
  lineSensorGainHandler.incrementIntegral(lineError);
  float derivative = lineError - lineSensorGainHandler.getLastError();

  float P = lineError;
  float I = lineSensorGainHandler.getIntegral();
  float D = derivative;

  float output = lineSensorGainHandler.getKp() * P +
                 lineSensorGainHandler.getKi() * I +
                 lineSensorGainHandler.getKd() * D;
  lineSensorGainHandler.setLastError(lineError);

  serialController.printWithTimestamp("PID: P(");
  Serial.print(P);
  Serial.print("), I(");
  Serial.print(I);
  Serial.print("), D(");
  Serial.print(D);
  Serial.print("), Output(");
  Serial.print(output);
  Serial.println(")");

  // Motor command calcs

  int desiredLeftSpeed = BASE_SPEED - output;
  int desiredRightSpeed = BASE_SPEED + output;

  if (PRINT_DESIRED_SPEEDS) {
    serialController.printWithTimestamp("Desired speeds: L(");
    Serial.print(desiredLeftSpeed);
    Serial.print("), R(");
    Serial.print(desiredRightSpeed);
    Serial.println(")");
  }

  // Calculate actual speed using encoders
  int actualLeftSpeed = sensorController.getEncoderBSpeed();
  int actualRightSpeed = sensorController.getEncoderASpeed();

  actualLeftSpeed = sensorController.speedAdjust(actualLeftSpeed, MIN_SPEED);
  actualRightSpeed = sensorController.speedAdjust(actualRightSpeed, MIN_SPEED);

  if (PRINT_ACTUAL_SPEEDS) {
    serialController.printWithTimestamp("Actual speeds: L(");
    Serial.print(actualLeftSpeed);
    Serial.print("), R(");
    Serial.print(actualRightSpeed);
    Serial.println(")");
  }

  // Adjust motor speed based on encoder feedback

  float kP_encoder = encoderGainHandler.getKp();

  int Enc_error = desiredLeftSpeed - actualLeftSpeed;
  int adjustedSpeed = desiredLeftSpeed + kP_encoder * Enc_error;
  int leftMotorSpeed =
      constrain(adjustedSpeed, LOWER_CONSTRAINT,
                CONSTRAINT); // Ensure speed stays within valid range

  Enc_error = desiredRightSpeed - actualRightSpeed;
  adjustedSpeed = desiredRightSpeed + kP_encoder * Enc_error;
  int rightMotorSpeed =
      constrain(adjustedSpeed, LOWER_CONSTRAINT,
                CONSTRAINT); // Ensure speed stays within valid range

  // Ensure minimum speed
  if (abs(leftMotorSpeed) < MIN_SPEED && leftMotorSpeed != 0) {
    leftMotorSpeed = (leftMotorSpeed < 0) ? -MIN_SPEED : MIN_SPEED;
  }
  if (abs(rightMotorSpeed) < MIN_SPEED && rightMotorSpeed != 0) {
    rightMotorSpeed = (rightMotorSpeed < 0) ? -MIN_SPEED : MIN_SPEED;
  }

  // Enable motors.
  motorController.servoDrive(MotorController::SERVO_A, rightMotorSpeed);
  motorController.servoDrive(MotorController::SERVO_B, leftMotorSpeed);

  if (PRINT_MOTOR_COMMAND) {
    serialController.printWithTimestamp("Motor command: L(");
    Serial.print(leftMotorSpeed);
    Serial.print("), R(");
    Serial.print(rightMotorSpeed);
    Serial.println(")");
  }

  serialController.printlnWithTimestamp("End of handleFollowLine.");
}

void handleRotation(MotorController::ROTATE_DIRECTION direction) {
  /*if (motorController.rotateRobot(direction,
                                  sensorController.getLineResultA())) {
    systemStateHandler.advanceStateFlowIndex();
  }*/
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

void handleLift(int direction) {
  const int LIFT_SPEED = 64;    // from 0 to 255
  const int LIFT_DISTANCE = 50; // steps
  motorController.stepperMotorB.setSpeed(LIFT_SPEED);

  switch (direction) {
  case MotorController::UP:
    // Move the lift up
    motorController.rotateStepperBsteps(LIFT_DISTANCE * 2);
    break;
  case MotorController::DOWN:
    // Move the lift down
    motorController.rotateStepperBsteps(-LIFT_DISTANCE);
    break;
  default:
    logError("Invalid direction");
    break;
  }
}

void handleLookAheadLineFollow() {
  int newSensorData = sensorController.getFullIRReadingResults(true);

  // Collect sensor data and store it in a buffer
  lookAhead.collectSensorData(newSensorData);

  // Get the look-ahead point based on the look-ahead distance.
  LookAhead::Point lookAheadPoint =
      lookAhead.getLookAheadPoint(LA_LOOK_AHEAD_DISTANCE);

  // Calculate error (distance from center)
  float error = lookAheadPoint.x;

  // PID control for line following
  float pidOutput = lookAhead.PID(error);

  // Calculate desired motor speeds
  int desiredLeftSpeed = LA_BASE_SPEED + pidOutput;
  int desiredRightSpeed = LA_BASE_SPEED - pidOutput;

  // Calculate actual speed using encoders
  int actualLeftSpeed = sensorController.getEncoderBSpeed();
  int actualRightSpeed = sensorController.getEncoderASpeed();

  actualLeftSpeed = sensorController.speedAdjust(actualLeftSpeed, MIN_SPEED);
  actualRightSpeed = sensorController.speedAdjust(actualRightSpeed, MIN_SPEED);

  // Adjust motor speeds based on encoder feedback
  float kP_encoder = encoderGainHandler.getKp();

  int Enc_error = desiredLeftSpeed - actualLeftSpeed;
  int adjustedSpeed = desiredLeftSpeed + kP_encoder * Enc_error;
  int leftMotorSpeed = constrain(adjustedSpeed, LOWER_CONSTRAINT, CONSTRAINT);

  Enc_error = desiredRightSpeed - actualRightSpeed;
  adjustedSpeed = desiredRightSpeed + kP_encoder * Enc_error;
  int rightMotorSpeed = constrain(adjustedSpeed, LOWER_CONSTRAINT, CONSTRAINT);

  // Ensure minimum speed
  if (abs(leftMotorSpeed) < MIN_SPEED && leftMotorSpeed != 0) {
    leftMotorSpeed = (leftMotorSpeed < 0) ? -MIN_SPEED : MIN_SPEED;
  }
  if (abs(rightMotorSpeed) < MIN_SPEED && rightMotorSpeed != 0) {
    rightMotorSpeed = (rightMotorSpeed < 0) ? -MIN_SPEED : MIN_SPEED;
  }

  // Enable motors with adjusted speeds
  motorController.servoDrive(MotorController::SERVO_A, rightMotorSpeed);
  motorController.servoDrive(MotorController::SERVO_B, leftMotorSpeed);

  serialController.printWithTimestamp("Motor command: L(");
  Serial.print(leftMotorSpeed);
  Serial.print("), R(");
  Serial.print(rightMotorSpeed);
  Serial.println(")");

  delay(10); // Small delay to stabilize the loop
}

// ===== HELPER FUNCTIONS =====

void logError(const char *message) {
  systemStateHandler.changeState(SystemState::IDLE);
  sensorController.turnLED(SensorController::ON);
  motorController.servosOff();
  serialController.printWithTimestamp(message);
  while (true)
    ; // Stop the program
}

void resetAllPIDMemory() {
  lineSensorGainHandler.reset();
  encoderGainHandler.reset();
}

void calculateRotation(int rotationType, int targetLocation) {
  MotorController::ROTATE_DIRECTION directionToRotate =
      motorController.getDirectionToRotate(targetLocation);
  SystemState::State outputRotationState = SystemState::ROTATE_LEFT;

  if (rotationType == TOWARDS && directionToRotate == MotorController::LEFT) {
    // If ( TOWARDS, LEFT ), go left.
    outputRotationState = SystemState::ROTATE_LEFT;

  } else if (rotationType == AWAY_FROM &&
             directionToRotate == MotorController::LEFT) {
    // If ( AWAY_FROM, LEFT ), go right.
    outputRotationState = SystemState::ROTATE_RIGHT;

  } else if (rotationType == TOWARDS &&
             directionToRotate == MotorController::RIGHT) {
    // If ( TOWARDS, RIGHT ), go right.
    outputRotationState = SystemState::ROTATE_RIGHT;

  } else if (rotationType == AWAY_FROM &&
             directionToRotate == MotorController::RIGHT) {
    // If ( AWAY_FROM, RIGHT ), go left.
    outputRotationState = SystemState::ROTATE_LEFT;

  } else {
    logError("Invalid rotation type");
  }

  systemStateHandler.changeState(outputRotationState);
}