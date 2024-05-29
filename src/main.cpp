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
                       LINE_FOLLOW_REGULAR_KI); // Line sensor gain handler
ControlGainHandler encoderGainHandler =
    ControlGainHandler(ENCODER_DRIVE_KP, ENCODER_DRIVE_KD,
                       ENCODER_DRIVE_KI);      // Encoder gain handler
BranchHandler branchHandler = BranchHandler(); // Branch handler
bool doLinePID = true;

// ===== ENUMS =====
enum LED_STATE { OFF = LOW, ON = HIGH };

enum BUTTON_STATE { PRESSED = HIGH, UNPRESSED = LOW };

enum LINE_FOLLOW_MODE { PICKUP, REGULAR, DROPOFF };

enum FOUR_BAR_DIRECTION { LOAD, UNLOAD };

enum LIFT_DIRECTION { DOWN, UP };

enum ROTATE_TYPE { TOWARDS, AWAY_FROM };

// ===== FUNCTION PROTOTYPES =====
void handleTest();
void handleIdle();
void handleIRIdle();
void handleUltraSonicIdle();
void handleFollowLine(int mode);
void initializePins();
void initializeSerialPort();
void turnLED(LED_STATE state);
BUTTON_STATE getBUTTON_STATE();
void logError(const char *message);
void handleCalibrate(MotorController::COMPONENT componentCode);
void handlePIDEncoderDrive(int BASE_SPEED);
void handleFourBar(int direction);
void resetAllPIDMemory();
void handleRotation(MotorController::ROTATE_DIRECTION direction);
void handleUltrasonicApproach();
void handleUltrasonicReverse();
void handleLift(int direction);
void calculateRotation(int rotationType, int targetLocation);
void buttonCheck();
void printTime();
void printWithTimestamp(const char *message);
void printlnWithTimestamp(const char *message);

long lastPrintTime = 0;
long lastIntegralRestTime = 0;

SystemState::State DEFAULT_STATE = SystemState::IDLE;
MotorController::COMPONENT CALIBRATE_COMPONENT = MotorController::BOTH_WHEELS;

LED_STATE currentLEDstate = OFF;

// ===== MAIN SETUP =====
void setup() {

  delay(1000);

  initializePins();
  initializeSerialPort();
  motorController.attachServoMotors();
  motorController.setStepperMotorSpeedsToMax();
  sensorController.zeroEncoders();

  turnLED(OFF);

  printlnWithTimestamp("Starting...");

  systemStateHandler.changeState(DEFAULT_STATE);
}

// ===== MAIN LOOP =====
void loop() {
  if (systemStateHandler.isNewStateFlowIndex()) {
    printlnWithTimestamp("STATE CHANGE");
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
      printlnWithTimestamp("changeState call.");
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
    buttonCheck();
    handleFollowLine(PICKUP);
    break;
  case SystemState::LINE_FOLLOW_DROPOFF:
    // Code for line following in dropoff mode
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
    handleLift(DOWN);
    break;
  case SystemState::LIFT_RAISE:
    // Raise the lift.
    handleLift(UP);
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
    printlnWithTimestamp("Exiting idle phase...");
    while (getBUTTON_STATE() == PRESSED)
      ;
    systemStateHandler.advanceStateFlowIndex();
  }
}

void handleCalibrate(MotorController::COMPONENT componentCode) {
  switch (componentCode) {
  case MotorController::FOUR_BAR:
    if (getBUTTON_STATE() == PRESSED) {
      // Hold down button until four-bar crank is in lowest position.
      turnLED(ON);
      motorController.rotateStepperAsteps(20);
      delay(100);
    } else {
      turnLED(OFF);
    }
    break;
  case MotorController::LIFT:
    if (getBUTTON_STATE() == PRESSED) {
      // Hold down button until lift is in lowest position.
      turnLED(ON);
      motorController.rotateStepperBsteps(20);
      delay(200);
    } else {
      turnLED(OFF);
    }
    break;
  case MotorController::RIGHT_WHEEL:
    if (getBUTTON_STATE() == PRESSED) {
      motorController.motorDriver.motorAForward(255); // full speed
      // int actualRightSpeed = sensorController.getEncoderASpeed();
      // Serial.print("Actual right speed: ");
      // Serial.println(actualRightSpeed);
      turnLED(ON);
    } else {
      motorController.servosOff();
      turnLED(OFF);
    }
    break;
  case MotorController::LEFT_WHEEL:
    if (getBUTTON_STATE() == PRESSED) {
      motorController.motorDriver.motorBForward(255); // full speed
      // int actualLeftSpeed = sensorController.getEncoderBSpeed();
      // Serial.print("Actual left speed: ");
      // Serial.println(actualLeftSpeed);
      turnLED(ON);
    } else {
      motorController.servosOff();
      turnLED(OFF);
    }
    break;
  case MotorController::BOTH_WHEELS:
    delay(1000);
    motorController.servoDrive(MotorController::SERVO_A, -255);
    motorController.servoDrive(MotorController::SERVO_B, -255);
    while (getBUTTON_STATE() == PRESSED) {
      motorController.servosOff();
      while (true)
        ;
    }
    break;
  default:
    logError("Invalid component code");
    break;
  }
}

void handleIRIdle() {
  /*
  // Read the sensor values
  sensorController.readLineSensorA();
  sensorController.readLineSensorB();

  int resultsA = sensorController.getLineResultA();
  int resultsB = sensorController.getLineResultB();

  Serial.print("Line sensors: [");
  Serial.print(sensorController.lineSensorA1.average());
  Serial.print(", ");
  Serial.print(sensorController.lineSensorA2.average());
  Serial.print(", ");
  Serial.print(sensorController.lineSensorA3.average());
  Serial.print("], [");
  Serial.print(sensorController.lineSensorB1.average());
  Serial.print(", ");
  Serial.print(sensorController.lineSensorB2.average());
  Serial.print(", ");
  Serial.print(sensorController.lineSensorB3.average());
  Serial.print("], { ");
  Serial.print(resultsA, BIN);
  Serial.print(", ");
  Serial.print(resultsB, BIN);
  Serial.println(" }");

  motorController.servosOff();*/
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

  bool SHOW_RAW_IR_READINGS = true;
  bool PRINT_DESIRED_SPEEDS = true;
  bool PRINT_ACTUAL_SPEEDS = true;
  bool PRINT_MOTOR_COMMAND = true;

  printlnWithTimestamp("Start of handleFollowLine.");
  // Read the sensor values
  int lineSensorReadingA1 = sensorController.lineSensorA1.cleanRead();
  int lineSensorReadingA2 = sensorController.lineSensorA2.cleanRead();
  int lineSensorReadingA3 = sensorController.lineSensorA3.cleanRead();
  int lineSensorReadingB1 = sensorController.lineSensorB1.cleanRead();
  int lineSensorReadingB2 = sensorController.lineSensorB2.cleanRead();
  int lineSensorReadingB3 = sensorController.lineSensorB3.cleanRead();
  // Combine.
  int lineSensorResults = sensorController.combineLineResult(
      lineSensorReadingA1, lineSensorReadingA2, lineSensorReadingA3,
      lineSensorReadingB1, lineSensorReadingB2, lineSensorReadingB3);

  // Debug messages.
  printWithTimestamp("Line sensors: ");
  if (SHOW_RAW_IR_READINGS) {
    Serial.print("[");
    Serial.print(lineSensorReadingA1);
    Serial.print(", ");
    Serial.print(lineSensorReadingA2);
    Serial.print(", ");
    Serial.print(lineSensorReadingA3);
    Serial.print("], [");
    Serial.print(lineSensorReadingB1);
    Serial.print(", ");
    Serial.print(lineSensorReadingB2);
    Serial.print(", ");
    Serial.print(lineSensorReadingB3);
    Serial.print("], ");
  }

  Serial.print("{ ");
  Serial.print(lineSensorResults, BIN);
  Serial.println(" }");
  /*
    switch (mode) {
    case PICKUP: { // Code for line following in pickup mode

      int targetLocation;
      switch (branchHandler.getTargetNum()) {
      case 1:
        // Serial.println("<First target>");
        targetLocation = PICKUP_LOCATION_1;
        break;
      case 2:
        // Serial.println("<Second target>");
        targetLocation = PICKUP_LOCATION_2;
      default:
        logError("Invalid target number");
        break;
      }

      int targetBranchNum =
          branchHandler.getTargetBranchNumFromLocation(targetLocation);

      // Check if the robot is currently over a branch.
      branchHandler.doBranchCheck(lineSensorResults);
      bool atBranch = branchHandler.getIsCurrentlyOverBranch();
      int branchNum = branchHandler.getCurrentLocation();

      if ((millis() - systemStateHandler.getLastStateChangeTime()) > 1000 &&
          atBranch && (branchNum == targetBranchNum)) {
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
    */

  if (millis() - lastIntegralRestTime > 5000) {
    lineSensorGainHandler.resetIntegral();
    lastIntegralRestTime = millis();
  }

  // PID Line stuff...
  int lineError = sensorController.determineError(lineSensorResults);

  printWithTimestamp("Line error: ");
  Serial.println(lineError);

  // =====================================
  // ===== EMERGENCY REVERSE COMMAND =====
  if (lineError == 99) {
    printlnWithTimestamp("<!> Reverse command");
    motorController.servosOff();
    motorController.servoDrive(MotorController::SERVO_A, -REVERSE_SPEED);
    motorController.servoDrive(MotorController::SERVO_B, -REVERSE_SPEED);
    delay(400);
    return;
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

  printWithTimestamp("PID: P(");
  Serial.print(P);
  Serial.print("), I(");
  Serial.print(I);
  Serial.print("), D(");
  Serial.print(D);
  Serial.print("), Output(");
  Serial.print(output);
  Serial.println(")");

  // Motor command calcs
  int BASE_SPEED = 160;
  int CONSTRAINT = 255;
  int LOWER_CONSTRAINT = -100;
  float MIN_SPEED = 80;

  int desiredLeftSpeed = BASE_SPEED - output;
  int desiredRightSpeed = BASE_SPEED + output;

  if (PRINT_DESIRED_SPEEDS) {
    printWithTimestamp("Desired speeds: L(");
    Serial.print(desiredLeftSpeed);
    Serial.print("), R(");
    Serial.print(desiredRightSpeed);
    Serial.println(")");
  }

  // Calculate actual speed using encoders
  int actualLeftSpeed = sensorController.getEncoderBSpeed();
  int actualRightSpeed = sensorController.getEncoderASpeed();

  actualLeftSpeed = sensorController.speedAdjust(actualLeftSpeed, CONSTRAINT);
  actualRightSpeed = sensorController.speedAdjust(actualRightSpeed, CONSTRAINT);

  if (PRINT_ACTUAL_SPEEDS) {
    printWithTimestamp("Actual speeds: L(");
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
  if (abs(leftMotorSpeed) < MIN_SPEED) {
    leftMotorSpeed = (leftMotorSpeed < 0) ? -MIN_SPEED : MIN_SPEED;
  }
  if (abs(rightMotorSpeed) < MIN_SPEED) {
    rightMotorSpeed = (rightMotorSpeed < 0) ? -MIN_SPEED : MIN_SPEED;
  }

  // Enable motors.
  motorController.servoDrive(MotorController::SERVO_A, rightMotorSpeed);
  motorController.servoDrive(MotorController::SERVO_B, leftMotorSpeed);

  if (PRINT_MOTOR_COMMAND) {
    printWithTimestamp("Motor command: L(");
    Serial.print(leftMotorSpeed);
    Serial.print("), R(");
    Serial.print(rightMotorSpeed);
    Serial.println(")");
  }

  printlnWithTimestamp("End of handleFollowLine.");
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
  case UP:
    // Move the lift up
    motorController.rotateStepperBsteps(LIFT_DISTANCE * 2);
    break;
  case DOWN:
    // Move the lift down
    motorController.rotateStepperBsteps(-LIFT_DISTANCE);
    break;
  default:
    logError("Invalid direction");
    break;
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
  Serial.begin(115200); // Initialize Serial port
  while (!Serial)
    ; // Waits for the Serial port to connect.
}

void turnLED(LED_STATE state) {
  if (currentLEDstate != state) {
    currentLEDstate = state;
    digitalWrite(LED_PIN, state);
  } else {
    // do nothing.
  }
}

BUTTON_STATE getBUTTON_STATE() {
  return (!digitalRead(BUTTON_PIN)) == HIGH ? PRESSED : UNPRESSED;
}

void logError(const char *message) {
  systemStateHandler.changeState(SystemState::IDLE);
  turnLED(ON);
  motorController.servosOff();
  printWithTimestamp(message);
  while (true)
    ; // Stop the program
}

void resetAllPIDMemory() {
  lineSensorGainHandler.reset();
  encoderGainHandler.reset();
}

void calculateRotation(int rotationType, int targetLocation) {
  if (rotationType == TOWARDS) {
    if (motorController.getDirectionToRotate(targetLocation) ==
        MotorController::LEFT) {
      systemStateHandler.changeState(SystemState::ROTATE_LEFT);
    } else {
      systemStateHandler.changeState(SystemState::ROTATE_RIGHT);
    }
  } else if (rotationType == AWAY_FROM) {
    if (motorController.getDirectionToRotate(targetLocation) ==
        MotorController::LEFT) {
      systemStateHandler.changeState(SystemState::ROTATE_RIGHT);
    } else {
      systemStateHandler.changeState(SystemState::ROTATE_LEFT);
    }
  } else {
    logError("Invalid rotation type");
  }
}

void buttonCheck() {
  if (getBUTTON_STATE() == PRESSED) {
    // Button is pressed
    turnLED(ON);
    logError("Button pressed");
  } else {
    // Button is not pressed
    turnLED(OFF);
  }
}

void printTime() {
  Serial.print("...Time: ");
  Serial.print(millis() - lastPrintTime);
  lastPrintTime = millis();
}

void printWithTimestamp(const char *message) {
  Serial.print("< time: ");
  Serial.print(millis() - lastPrintTime);
  lastPrintTime = millis();
  Serial.print(" > ");
  Serial.print(message);
}

void printlnWithTimestamp(const char *message) {
  Serial.print("< time: ");
  Serial.print(millis() - lastPrintTime);
  lastPrintTime = millis();
  Serial.print(" > ");
  Serial.println(message);
}