/**
 * @file main.cpp
 * @brief This file contains the main Arduino sketch for the MAE_162_Robot
 * project.
 *
 * This sketch initializes various libraries and objects, and defines the main
 * setup() and loop() functions. It also includes function prototypes and global
 * variables used throughout the sketch. The main loop() function handles the
 * state flow of the system and calls the appropriate functions based on the
 * current state. The handleTest() function is used for testing purposes.
 *
 * @note This code is specific to the MAE_162_Robot project and assumes the
 * presence of certain libraries and hardware components.
 *
 * @author Austin D'Acquisto
 *
 * @date June 3, 2024
 */
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
SystemState::State DEFAULT_STATE = SystemState::CALIBRATE;

MotorController::COMPONENT CALIBRATE_COMPONENT = MotorController::SINGLE_SERVO;
MotorController::SERVO CALIBRATE_SERVO = MotorController::SERVO_FRONT_LEFT;
MotorController::MOTOR_DIRECTION CALIBRATE_DIRECTION = MotorController::FORWARD;

// ===== CONTROL OBJECTS =====
SystemStateHandler systemStateHandler =
    SystemStateHandler(); // System state handler

SerialController serialController = SerialController(); // Serial controller
MotorController motorController = MotorController();    // Motor controller
SensorController sensorController = SensorController(); // Sensor controller

BranchHandler branchHandler = BranchHandler(); // Branch handler
LookAhead lookAhead = LookAhead();             // Look-ahead controller

ControlGainHandler encoderFrontRight = ControlGainHandler(1.0, 0.0, 0.0);
ControlGainHandler encoderFrontLeft = ControlGainHandler(1.0, 0.0, 0.0);
ControlGainHandler encoderRearRight = ControlGainHandler(1.0, 0.0, 0.0);
ControlGainHandler encoderRearLeft = ControlGainHandler(1.0, 0.0, 0.0);

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
void handleRotation(MotorController::ROTATE_DIRECTION direction);
void handleUltrasonicApproach();
void handleUltrasonicReverse();
void handleLift(int direction);
void handleLookAheadLineFollow();
void ultrasonicTurnCheck();
void ultrasonicObstacleCheck();
void calibrateFourBar(int mode);
void drive(int leftSpeed, int rightSpeed);
void speedAdjustRobust(int &speed);
void calibrateAllWheels(MotorController::MOTOR_DIRECTION direction);
void calibrateSingleServo(MotorController::SERVO whichServo);

// Helper functions
void calculateRotation(int rotationType, int targetLocation);

// LOGGING
void logError(const char *message);

// ===== MAIN SETUP =====
/**
 * @brief Initializes the robot's setup.
 *
 * This function is called once when the microcontroller starts up. It
 * initializes various components of the robot, such as the serial controller,
 * sensor controller, motor controller, and system state handler. It also
 * introduces a delay of 1000 milliseconds before proceeding with the
 * initialization.
 */
void setup() {
  serialController.init();
  sensorController.init();
  motorController.init();
  motorController.disableSteppers();

  systemStateHandler.init(DEFAULT_STATE);
  systemStateHandler.setStateFlowIndex(0);
}

// ===== MAIN LOOP =====
/**
 * @brief The main loop function that runs repeatedly in the program.
 *
 * This function is responsible for executing the robot's state machine logic.
 * It checks the current state and performs the corresponding actions based on
 * the state flow index. It also handles error cases and executes specific code
 * for each state.
 */
void loop() {
  if (systemStateHandler.isNewStateFlowIndex()) {
    serialController.printlnWithTimestamp("STATE CHANGE");
    // On state change.
    motorController.servosOff();
    // motorController.setStepperMotorSpeedsToMax();
    //  branchHandler.reset();
    delay(2000);
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
    systemStateHandler.changeState(SystemState::FOUR_BAR_LOAD);
    // systemStateHandler.changeState(SystemState::ULTRASONIC_REVERSE);
    break;
  case 7:
    systemStateHandler.changeState(SystemState::LIFT_RAISE);
    // Rotate to face front again.
    // calculateRotation(AWAY_FROM, PICKUP_LOCATION_1);
    break;
  case 8:
    // Line follow to pickup location 2
    systemStateHandler.changeState(SystemState::FOUR_BAR_UNLOAD);
    // systemStateHandler.changeState(SystemState::LINE_FOLLOW_PICKUP);
    // branchHandler.setTargetNum(2);
    break;
  case 9:
    // Rotate to face pickup location 2
    while (true)
      ;
    // calculateRotation(TOWARDS, PICKUP_LOCATION_2);
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
    motorController.enableStepper(MotorController::STEPPER_FOUR_BAR);
    motorController.handleFourBar(MotorController::LOAD);
    motorController.disableSteppers();
    systemStateHandler.advanceStateFlowIndex();
    break;
  case SystemState::FOUR_BAR_UNLOAD:
    // Code for four-bar mechanism
    motorController.enableStepper(MotorController::STEPPER_FOUR_BAR);
    motorController.handleFourBar(MotorController::UNLOAD);
    motorController.disableSteppers();
    systemStateHandler.advanceStateFlowIndex();
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
    motorController.enableStepper(MotorController::STEPPER_LIFT);
    handleLift(MotorController::DOWN);
    motorController.disableSteppers();
    systemStateHandler.advanceStateFlowIndex();
    break;
  case SystemState::LIFT_RAISE:
    // Raise the lift.
    motorController.enableStepper(MotorController::STEPPER_LIFT);
    handleLift(MotorController::UP);
    motorController.disableSteppers();
    systemStateHandler.advanceStateFlowIndex();
    break;
  default:
    // Error handling
    logError("Invalid state");
    break;
  }
}

// ===== OTHER FUNCTIONS =====
/**
 * @brief Function to handle the test.
 *
 * This function performs a series of tests for motors and steppers.
 * It checks the button status and controls the LEDs accordingly.
 * It also controls the motors and steppers based on the elapsed time.
 *
 * @note This function assumes that the necessary objects and libraries are
 * properly initialized.
 */
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
      motorController.motorDriverFront.motorAForward();
      motorController.motorDriverFront.motorBStop();
    } else if (millis() - myTimerStart <
               MOTOR_A_REV_TEST_START + MOTOR_A_TEST_LENGTH) {
      Serial.print("Motor A Reverse");
      // Test motor A (right) REVERSE
      motorController.motorDriverFront.motorAReverse();
      motorController.motorDriverFront.motorBStop();
    } else if (millis() - myTimerStart <
               MOTOR_B_FWD_TEST_START + MOTOR_B_TEST_LENGTH) {
      Serial.print("Motor B Forward");
      // Test motor B (left) FORWARD
      motorController.motorDriverFront.motorBForward();
      motorController.motorDriverFront.motorAStop();
    } else if (millis() - myTimerStart <
               MOTOR_B_REV_TEST_START + MOTOR_B_TEST_LENGTH) {
      Serial.print("Motor B Reverse");
      // Test motor B (left) REVERSE
      motorController.motorDriverFront.motorBReverse();
      motorController.motorDriverFront.motorAStop();
    } else if (millis() - myTimerStart <
               STEPPER_A_FWD_TEST_START + STEPPER_A_TEST_LENGTH) {
      Serial.print("Stepper A Forward");
      // Test stepper A (four-bar) FORWARD
      motorController.enableStepper(MotorController::STEPPER_FOUR_BAR);
      motorController.servosOff();
      motorController.stepperDrive(
          MotorController::STEPPER_FOUR_BAR, 1,
          motorController.degToSteps(360, MotorController::STEPPER_FOUR_BAR));
      motorController.disableSteppers();
    } else if (millis() - myTimerStart <
               STEPPER_A_REV_TEST_START + STEPPER_A_TEST_LENGTH) {
      Serial.print("Stepper A Reverse");
      // Test stepper A (four-bar) REVERSE
      motorController.enableStepper(MotorController::STEPPER_FOUR_BAR);
      motorController.servosOff();
      motorController.stepperDrive(
          MotorController::STEPPER_FOUR_BAR, -1,
          motorController.degToSteps(360, MotorController::STEPPER_FOUR_BAR));
      motorController.disableSteppers();
    } else if (millis() - myTimerStart <
               STEPPER_B_FWD_TEST_START + STEPPER_B_TEST_LENGTH) {
      Serial.print("Stepper B Forward");
      // Test stepper B (lift) FORWARD
      motorController.enableStepper(MotorController::STEPPER_LIFT);
      motorController.servosOff();
      motorController.stepperDrive(
          MotorController::STEPPER_LIFT, 1,
          motorController.degToSteps(360, MotorController::STEPPER_LIFT));
      motorController.disableSteppers();
    } else if (millis() - myTimerStart <
               STEPPER_B_REV_TEST_START + STEPPER_B_TEST_LENGTH) {
      Serial.print("Stepper B Reverse");
      // Test stepper B (lift) REVERSE
      motorController.enableStepper(MotorController::STEPPER_LIFT);
      motorController.servosOff();
      motorController.stepperDrive(
          MotorController::STEPPER_LIFT, -1,
          motorController.degToSteps(360, MotorController::STEPPER_LIFT));
      motorController.disableSteppers();
    } else {
      Serial.print("Resetting...");
      delay(DELAY_BETWEEN_LOOPS);
      myTimerStart = millis();
    }
  }
  Serial.println();
}

/**
 * Handles the idle phase of the robot.
 * Turns off the servos, turns on the LED, and waits for a button press.
 * When the button is pressed, the LED is turned off, a message is printed to
 * the serial monitor, and the state flow index is advanced.
 */
void handleIdle() {
  // Disable everything.
  motorController.disableSteppers();
  motorController.servosOff();

  // Turn on LED.
  sensorController.turnLED(SensorController::ON);

  // Wait for button press.
  if (sensorController.readButton() == SensorController::PRESSED) {
    // Turn off LED.
    sensorController.turnLED(SensorController::OFF);

    // Print message.
    serialController.printlnWithTimestamp("Exiting idle phase...");

    // Wait for button release.
    while (sensorController.readButton() == SensorController::PRESSED)
      ;

    // Advance state flow index.
    systemStateHandler.advanceStateFlowIndex();
  }
}

/**
 * Handles the calibration process for different motor components.
 *
 * @param componentCode The code representing the motor component to calibrate.
 */
void handleCalibrate(MotorController::COMPONENT componentCode) {
  switch (componentCode) {
  case MotorController::FOUR_BAR: {
    int calibrateMode = 1;
    calibrateFourBar(calibrateMode);
    break;
  }
  case MotorController::LIFT: {
    int LIFT_CALIBRATION_MODE = 2;

    if (LIFT_CALIBRATION_MODE == 1) {
      if (sensorController.readButton() == SensorController::PRESSED) {
        // Do ten steps at a time.
        Serial.println("Calibrating lift... ");
        motorController.enableStepper(MotorController::STEPPER_LIFT);
        sensorController.turnLED(SensorController::ON);
        motorController.lowerLift(10);
        motorController.disableSteppers();
        delay(1000);
      } else {
        motorController.disableSteppers();
        sensorController.turnLED(SensorController::OFF);
      }
    } else if (LIFT_CALIBRATION_MODE == 2) {
      // Do the full range.
      if (sensorController.readButton() == SensorController::PRESSED) {
        motorController.enableStepper(MotorController::STEPPER_LIFT);
        sensorController.turnLED(SensorController::ON);
        motorController.lowerLift();
        motorController.disableSteppers();
        delay(1000);
      } else {
        motorController.disableSteppers();
        sensorController.turnLED(SensorController::OFF);
      }
    }
    break;
  }
  case MotorController::ALL_WHEELS:
    calibrateAllWheels(CALIBRATE_DIRECTION);
  case MotorController::SINGLE_SERVO:
    calibrateSingleServo(CALIBRATE_SERVO);
  default:
    logError("Invalid component code");
    break;
  }
}

void calibrateSingleServo(MotorController::SERVO whichServo) {
  motorController.servoDrive(whichServo, 255);
}

void calibrateAllWheels(MotorController::MOTOR_DIRECTION direction) {
  if (direction == MotorController::FORWARD) {
    drive(255, 255);
  } else if (direction == MotorController::BACKWARD) {
    drive(-255, -255);
  } else {
    logError("Invalid direction");
  }
}

void calibrateFourBar(int mode) {
  switch (mode) {
  case 1:
    if (sensorController.readButton() == SensorController::PRESSED) {
      // Hold down button until four-bar crank is in lowest position.
      sensorController.turnLED(SensorController::ON);
      motorController.enableStepper(MotorController::STEPPER_FOUR_BAR);
      motorController.stepperDrive(MotorController::STEPPER_FOUR_BAR,
                                   FOUR_BAR_SPEED, 1);
      // positive for loading.
      delay(30);
    } else {
      motorController.disableSteppers();
      sensorController.turnLED(SensorController::OFF);
    }
    break;
  case 2:
    if (sensorController.readButton() == SensorController::PRESSED) {
      // Do a full rotation.
      sensorController.turnLED(SensorController::ON);
      motorController.stepperDrive(
          MotorController::STEPPER_FOUR_BAR, FOUR_BAR_SPEED,
          motorController.degToSteps(360, MotorController::STEPPER_FOUR_BAR));
      delay(20);
    } else {
      sensorController.turnLED(SensorController::OFF);
    }
    break;
  }
}

/**
 * Function to handle the idle state of the IR sensor.
 * This function retrieves the full IR reading results from the sensor
 * controller, and optionally displays the raw IR readings and binary reading
 * mapping. It turns off the servos of the motor controller and introduces a
 * delay of 100ms.
 */
void handleIRIdle() {
  bool isNewSensor = true;

  if (isNewSensor) {
    uint8_t sensorReading = sensorController.getNewIRSensor().readSensorData();

    Serial.print("New IR value: ");
    uint8_t leastSignificantBits =
        sensorReading & 0x1F; // Masking to get the 5 least significant bits
    for (int i = 4; i >= 0; i--) { // Loop to print each bit from MSB to LSB
      Serial.print((leastSignificantBits >> i) & 0x01);
    }
    Serial.println(); // To move to the next line after printing the bits

    motorController.servosOff();
    delay(100);

  } else {
    bool SHOW_RAW_IR_READINGS = true;
    bool SHOW_BINARY_READING = true;

    int lineSensorResults =
        sensorController.getFullIRReadingResults(SHOW_RAW_IR_READINGS, false);

    if (SHOW_BINARY_READING) {
      serialController.printWithTimestamp("Sensor Mapping: { ");
      serialController.printBinaryWithLeadingZeros(lineSensorResults);
      Serial.println(" }\n");
    }

    motorController.servosOff();
    delay(100);
  }
}

/**
 * Function to handle the idle state of the ultrasonic sensor.
 * Reads the ultrasonic sensor and prints the distance.
 * Turns off the servos and adds a delay of 100 milliseconds.
 */
void handleUltraSonicIdle() {
  // Read the ultrasonic sensor
  long distance = sensorController.getUltrasonicHandler().getDist();

  // Print the distance
  Serial.print("Distance: ");
  Serial.println(distance);

  motorController.servosOff();

  delay(100);
}

/**
 * Handles the line following behavior based on the given mode.
 *
 * @param mode The mode of line following (PICKUP, REGULAR, DROPOFF).
 */
void handleFollowLine(int mode) {

  ultrasonicObstacleCheck();

  handleLookAheadLineFollow();

  return;

  // bool DO_ULTRASONIC_CHECK_OBSTACLE = false;
  // bool DO_ULTRASONIC_CHECK_TURN = false;
  // bool SHOW_RAW_IR_READINGS = true;
  // bool PRINT_DESIRED_SPEEDS = true;
  // bool PRINT_ACTUAL_SPEEDS = true;
  // bool PRINT_MOTOR_COMMAND = true;
  // bool DO_EMERGENCY_REVERSE = true;

  // serialController.printlnWithTimestamp("Start of handleFollowLine.");
  // // Read the sensor values

  // bool SHOW_BINARY_READING = true;

  // int lineSensorResults =
  //     sensorController.getFullIRReadingResults(SHOW_RAW_IR_READINGS,
  //     false);

  // if (SHOW_BINARY_READING) {
  //   serialController.printWithTimestamp("Sensor Mapping: { ");
  //   serialController.printBinaryWithLeadingZeros(lineSensorResults);
  //   Serial.println(" }\n");
  // }

  // switch (mode) {
  // case PICKUP: { // Code for line following in pickup mode
  //   // Check if at target branch.
  //   if ((millis() - systemStateHandler.getLastStateChangeTime()) > 1000 &&
  //       branchHandler.isAtTargetLocation()) {
  //     // STOP.
  //     Serial.println("<debug> At branch, stopping.");
  //     systemStateHandler.advanceStateFlowIndex();
  //     return;
  //   } else {
  //     // Follow the line.
  //   }
  //   break;
  // }
  // case REGULAR: {
  //   break;
  // }
  // case DROPOFF: {
  //   break;
  // }
  // default:
  //   logError("Invalid mode");
  //   break;
  // }

  // if (millis() - lineSensorGainHandler.getLastIntegralResetTime() > 5000) {
  //   lineSensorGainHandler.resetIntegral();
  // }

  // // PID Line stuff...
  // int lineError = -sensorController.determineError(lineSensorResults);

  // if (lineError == 0) {
  //   lineSensorGainHandler.resetIntegral();
  // }

  // serialController.printWithTimestamp("Line error: ");
  // Serial.println(lineError);

  // // ============================
  // // ===== ULTRASONIC CHECK =====s
  // if (DO_ULTRASONIC_CHECK_OBSTACLE) {
  //   if (sensorController.getUltrasonicHandler().isObstacle(
  //           OBSTACLE_DISTANCE_THRESHOLD)) {
  //     serialController.printlnWithTimestamp("<!> Obstacle detected.");
  //     motorController.servosOff();
  //     delay(100);
  //     return;
  //   }
  // } else if (DO_ULTRASONIC_CHECK_TURN) {
  //   ultrasonicTurnCheck();
  // }
  // // ============================
  // // ============================

  // // =====================================
  // // ===== EMERGENCY REVERSE COMMAND =====
  // if (DO_EMERGENCY_REVERSE && lineError == 99) {
  //   serialController.printlnWithTimestamp("<!> Reverse command");
  //   motorController.servosOff();
  //   motorController.servoDrive(MotorController::SERVO_A, -REVERSE_SPEED);
  //   motorController.servoDrive(MotorController::SERVO_B, -REVERSE_SPEED);
  //   delay(100);
  //   return;
  // } else if (!DO_EMERGENCY_REVERSE) {
  //   lineError = lineSensorGainHandler.getLastError();
  // }
  // // =====================================
  // // =====================================

  // // PID Line stuff cont'd...
  // lineSensorGainHandler.incrementIntegral(lineError);
  // float derivative = lineError - lineSensorGainHandler.getLastError();

  // float P = lineError;
  // float I = lineSensorGainHandler.getIntegral();
  // float D = derivative;

  // float output = lineSensorGainHandler.getKp() * P +
  //                lineSensorGainHandler.getKi() * I +
  //                lineSensorGainHandler.getKd() * D;
  // lineSensorGainHandler.setLastError(lineError);

  // serialController.printWithTimestamp("PID: P(");
  // Serial.print(P);
  // Serial.print("), I(");
  // Serial.print(I);
  // Serial.print("), D(");
  // Serial.print(D);
  // Serial.print("), Output(");
  // Serial.print(output);
  // Serial.println(")");

  // // Motor command calcs

  // int desiredLeftSpeed = BASE_SPEED - output;
  // int desiredRightSpeed = BASE_SPEED + output;

  // if (PRINT_DESIRED_SPEEDS) {
  //   serialController.printWithTimestamp("Desired speeds: L(");
  //   Serial.print(desiredLeftSpeed);
  //   Serial.print("), R(");
  //   Serial.print(desiredRightSpeed);
  //   Serial.println(")");
  // }

  // // Calculate actual speed using encoders
  // int actualLeftSpeed = sensorController.getEncoderBSpeed();
  // int actualRightSpeed = sensorController.getEncoderASpeed();

  // actualLeftSpeed = sensorController.speedAdjust(actualLeftSpeed,
  // MIN_SPEED); actualRightSpeed =
  // sensorController.speedAdjust(actualRightSpeed, MIN_SPEED);

  // if (PRINT_ACTUAL_SPEEDS) {
  //   serialController.printWithTimestamp("Actual speeds: L(");
  //   Serial.print(actualLeftSpeed);
  //   Serial.print("), R(");
  //   Serial.print(actualRightSpeed);
  //   Serial.println(")");
  // }

  // // Adjust motor speed based on encoder feedback

  // float kP_encoder = encoderGainHandler.getKp();

  // int Enc_error = desiredLeftSpeed - actualLeftSpeed;
  // int adjustedSpeed = desiredLeftSpeed + kP_encoder * Enc_error;
  // int leftMotorSpeed =
  //     constrain(adjustedSpeed, LOWER_CONSTRAINT,
  //               CONSTRAINT); // Ensure speed stays within valid range

  // Enc_error = desiredRightSpeed - actualRightSpeed;
  // adjustedSpeed = desiredRightSpeed + kP_encoder * Enc_error;
  // int rightMotorSpeed =
  //     constrain(adjustedSpeed, LOWER_CONSTRAINT,
  //               CONSTRAINT); // Ensure speed stays within valid range

  // // Ensure minimum speed
  // if (abs(leftMotorSpeed) < MIN_SPEED && leftMotorSpeed != 0) {
  //   leftMotorSpeed = (leftMotorSpeed < 0) ? -MIN_SPEED : MIN_SPEED;
  // }
  // if (abs(rightMotorSpeed) < MIN_SPEED && rightMotorSpeed != 0) {
  //   rightMotorSpeed = (rightMotorSpeed < 0) ? -MIN_SPEED : MIN_SPEED;
  // }

  // // Enable motors.
  // motorController.servoDrive(MotorController::SERVO_A, rightMotorSpeed);
  // motorController.servoDrive(MotorController::SERVO_B, leftMotorSpeed);

  // if (PRINT_MOTOR_COMMAND) {
  //   serialController.printWithTimestamp("Motor command: L(");
  //   Serial.print(leftMotorSpeed);
  //   Serial.print("), R(");
  //   Serial.print(rightMotorSpeed);
  //   Serial.println(")");
  // }

  // serialController.printlnWithTimestamp("End of handleFollowLine.");
}

/**
 * Handles the rotation of the robot in the specified direction.
 *
 * @param direction The direction in which the robot should rotate.
 */
void handleRotation(MotorController::ROTATE_DIRECTION direction) {
  /*if (motorController.rotateRobot(direction,
                                  sensorController.getLineResultA())) {
    systemStateHandler.advanceStateFlowIndex();
  }*/
}

/**
 * Handles the ultrasonic approach behavior.
 * Reads the ultrasonic sensor, prints the distance, and performs actions
 * based on the distance.
 */
void handleUltrasonicApproach() {
  // Read the ultrasonic sensor
  int DISTANCE_THRESHOLD = 10;
  long distance = sensorController.getUltrasonicHandler().getDist();

  if (sensorController.getUltrasonicHandler().getMemory() == 0) {
    sensorController.getUltrasonicHandler().setMemory(distance);
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

/**
 * Handles the reverse movement based on the ultrasonic sensor reading.
 * If the distance measured by the ultrasonic sensor is less than the stored
 * ultrasonic memory, the robot will move in reverse at a predefined speed.
 * Otherwise, the motor servos will be turned off and the state flow index
 * will be advanced.
 */
void handleUltrasonicReverse() {
  // int REVERSE_SPEED = 64;

  long distance = sensorController.getUltrasonicHandler().getDist();

  // Print the distance
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < sensorController.getUltrasonicHandler().getMemory()) {
    // handlePIDEncoderDrive(-REVERSE_SPEED);
    ;
  } else {
    motorController.servosOff();
    systemStateHandler.advanceStateFlowIndex();
  }
}

/**
 * Handles the lift movement based on the specified direction.
 *
 * @param direction The direction of the lift movement. Should be
 * MotorController::UP or MotorController::DOWN.
 */
void handleLift(int direction) {
  switch (direction) {
  case MotorController::UP:
    // Move the lift up
    motorController.raiseLift();
    break;
  case MotorController::DOWN:
    // Move the lift down
    motorController.lowerLift();
    break;
  default:
    logError("Invalid direction");
    break;
  }
}

/**
 * Performs line following using look-ahead control.
 * This function reads sensor data, performs linear regression on the data
 * points, calculates the error, applies PID control, adjusts motor speeds
 * based on encoder feedback, and drives the motors accordingly.
 */
void handleLookAheadLineFollow() {
  // Check for a 90 degree turn based on the ultrasonic sensor readings.
  ultrasonicTurnCheck();

  // Decide whether to use the new IR sensor.
  bool doNewIR = true;

  // Read the sensor data.
  uint8_t newSensorDataUint8 =
      sensorController.getNewIRSensor().readSensorData();

  // Add the new binary number to the buffer.
  lookAhead.addSensorReading(newSensorDataUint8, doNewIR);

  // Define a 2D array to store the points.
  float points[LA_BUFFER_SIZE][2];

  // Define a variable to store the number of points.
  int num_points;

  // Get the points from the buffer.
  lookAhead.getPoints(points, num_points, doNewIR);

  // Define variables to store the slope and intercept.
  float slope, intercept;

  // Define the number of recent points to use for regression.
  int recent_points = 6; // Tunable value

  // Perform linear regression on the points.
  bool doForward = lookAhead.linearRegression(points, num_points, recent_points,
                                              slope, intercept);

  // Define a y-value to predict the x-value for.
  float y_value = 5.0;

  // Predict the x-value for the given y-value.
  float predicted_x = lookAhead.predictX(slope, intercept, y_value);

  // Calculate the error.
  float error = predicted_x;

  // PID control for line following
  float pidOutput = lookAhead.PID(error);

  // Calculate desired motor speeds
  int desiredLeftSpeed = LA_BASE_SPEED + pidOutput;
  int desiredRightSpeed = LA_BASE_SPEED - pidOutput;

  if (!doForward) {
    Serial.println("<!> Reverse command");
    motorController.getLastDesiredSpeeds(desiredLeftSpeed, desiredRightSpeed);
  } else {
    motorController.setLastDesiredSpeeds(desiredLeftSpeed, desiredRightSpeed);
  }

  // Switch the speeds.
  if (doNewIR) {
    int temp = desiredLeftSpeed;
    desiredLeftSpeed = desiredRightSpeed;
    desiredRightSpeed = temp;
  }

  // Adjust the motor speeds.
  drive(desiredLeftSpeed, desiredRightSpeed);
}

// ===== HELPER FUNCTIONS =====

/**
 * Checks if a 90 degree turn is required based on the ultrasonic sensor
 * readings. If a turn is required, it stops the servo motors and enters an
 * infinite loop. This function has a grace period of 40 seconds before
 * checking for a turn.
 */
void ultrasonicTurnCheck() {
  int numSecondsGracePeriod = 40;
  if (millis() - systemStateHandler.getLastStateChangeTime() <
      (unsigned long)(numSecondsGracePeriod * 1000)) {
    return;
  }
  if (sensorController.getUltrasonicHandler().isObstacle(
          NINETY_DEGREE_TURN_DISTANCE)) {
    serialController.printlnWithTimestamp("<!> 90 degree turn detected.");
    motorController.servosOff();
    while (true)
      ;
    delay(100);
    return;
  }
}

void ultrasonicObstacleCheck() {
  int numSecondsGracePeriod = 1;
  if (millis() - systemStateHandler.getLastStateChangeTime() <
      (unsigned long)(numSecondsGracePeriod * 1000)) {
    return;
  }
  if (sensorController.getUltrasonicHandler().isObstacle(
          OBSTACLE_DISTANCE_THRESHOLD)) {
    serialController.printlnWithTimestamp("<!> Obstacle detected.");
    motorController.servosOff();
    delay(100);
    return;
  }
}

/**
 * Logs an error message and performs necessary actions to handle the error.
 *
 * @param message The error message to be logged.
 */
void logError(const char *message) {
  systemStateHandler.changeState(SystemState::IDLE);
  sensorController.turnLED(SensorController::ON);
  motorController.servosOff();
  serialController.printWithTimestamp(message);
  while (true)
    ; // Stop the program
}

/**
 * Calculates the rotation direction based on the rotation type and target
 * location. Updates the system state accordingly.
 *
 * @param rotationType The type of rotation (TOWARDS or AWAY_FROM).
 * @param targetLocation The target location for rotation.
 */
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

void drive(int leftSpeed, int rightSpeed) {
  Serial.print("Desired motor commands: ");
  Serial.print(leftSpeed);
  Serial.print(", ");
  Serial.println(rightSpeed);

  // Measure encoder speeds.
  float encFrontRightSpeed = sensorController.getEncFrontRightSpeed();
  float encFrontLeftSpeed = sensorController.getEncFrontLeftSpeed();
  float encRearRightSpeed = sensorController.getEncRearRightSpeed();
  float encRearLeftSpeed = sensorController.getEncRearLeftSpeed();

  // Calculate encoder speed errors.
  float encFrontRightErr = rightSpeed - encFrontRightSpeed;
  float encFrontLeftErr = leftSpeed - encFrontLeftSpeed;
  float encRearRightErr = rightSpeed - encRearRightSpeed;
  float encRearLeftErr = leftSpeed - encRearLeftSpeed;

  // Calculate PID corrections.
  float encFrontRightCorr = encoderFrontRight.calculatePID(encFrontRightErr);
  float encFrontLeftCorr = encoderFrontLeft.calculatePID(encFrontLeftErr);
  float encRearRightCorr = encoderRearRight.calculatePID(encRearRightErr);
  float encRearLeftCorr = encoderRearLeft.calculatePID(encRearLeftErr);

  // Adjust speeds.
  int frontRightAdjusted = rightSpeed + encFrontRightCorr;
  int frontLeftAdjusted = leftSpeed + encFrontLeftCorr;
  int rearRightAdjusted = rightSpeed + encRearRightCorr;
  int rearLeftAdjusted = leftSpeed + encRearLeftCorr;

  // Ensure speeds are within constraints.
  speedAdjustRobust(frontRightAdjusted);
  speedAdjustRobust(frontLeftAdjusted);
  speedAdjustRobust(rearRightAdjusted);
  speedAdjustRobust(rearLeftAdjusted);

  // Debug messages.
  Serial.print("EncFrontRight: ");
  Serial.print(encFrontRightSpeed);
  Serial.print(" is off by ");
  Serial.print(encFrontRightErr);
  Serial.print(" and corrected by ");
  Serial.println(encFrontRightCorr);

  Serial.print("EncFrontLeft: ");
  Serial.print(encFrontLeftSpeed);
  Serial.print(" is off by ");
  Serial.print(encFrontLeftErr);
  Serial.print(" and corrected by ");
  Serial.println(encFrontLeftCorr);

  Serial.print("EncRearRight: ");
  Serial.print(encRearRightSpeed);
  Serial.print(" is off by ");
  Serial.print(encRearRightErr);
  Serial.print(" and corrected by ");
  Serial.println(encRearRightCorr);

  Serial.print("EncRearLeft: ");
  Serial.print(encRearLeftSpeed);
  Serial.print(" is off by ");
  Serial.print(encRearLeftErr);
  Serial.print(" and corrected by ");
  Serial.println(encRearLeftCorr);

  // Drive motors.
  motorController.servoDrive(MotorController::SERVO_FRONT_RIGHT,
                             frontRightAdjusted);
  motorController.servoDrive(MotorController::SERVO_FRONT_LEFT,
                             frontLeftAdjusted);
  motorController.servoDrive(MotorController::SERVO_REAR_RIGHT,
                             rearRightAdjusted);
  motorController.servoDrive(MotorController::SERVO_REAR_LEFT,
                             rearLeftAdjusted);

  // Log speeds.
  Serial.print("Actual motor commands: ");
  Serial.print(frontRightAdjusted);
  Serial.print(", ");
  Serial.print(frontLeftAdjusted);
  Serial.print(", ");
  Serial.print(rearRightAdjusted);
  Serial.print(", ");
  Serial.println(rearLeftAdjusted);
}

void speedAdjustRobust(int &speed) {
  // Speed = 0 case.
  if (speed == 0) {
    return;
  }

  // Determine direction.
  bool backwards = false;
  if (speed < 0) {
    backwards = true;
    speed = -speed;
  }

  // Ensure minimum speed is met.
  if (speed < LA_MIN_SPEED) {
    speed = LA_MIN_SPEED;
  }

  if (backwards) {
    speed = -speed;
  }

  // Ensure requested speed is within constraints.
  if (speed > LA_UPPER_CONSTRAINT) {
    speed = LA_UPPER_CONSTRAINT;
  } else if (speed < LA_LOWER_CONSTRAINT) {
    speed = LA_LOWER_CONSTRAINT;
  }
}