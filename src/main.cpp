// This is the main Arduino library, which provides the core functionality for
// Arduino.
#include <Arduino.h>
// This library provides an interface for controlling motors using the DRV8833
// motor driver.
#include "DRV8833.h"
// This library provides an interface for controlling stepper motors.
#include "Stepper.h"
// This is the configuration file for this project. It contains definitions for
// various constants and settings.
#include "config.h"
// This library provides an interface for reading rotary encoders.
#include "Encoder.h"
// This library provides an interface for managing the state of the system.
#include "SystemStateHandler.h"
// This library provides an interface for reading line sensors.
#include "MovingAverageSensor.h"

// ===== GLOBAL VARIABLES =====
DRV8833 motorDriver = DRV8833(); // Motor driver
Stepper stepperMotorA(STEPPER_A_STEPS_PER_REVOLUTION, STEPPER_PIN_A1,
                      STEPPER_PIN_A2, STEPPER_PIN_A3,
                      STEPPER_PIN_A4); // Stepper motor A (four-bar)
Stepper stepperMotorB(STEPPER_B_STEPS_PER_REVOLUTION, STEPPER_PIN_B1,
                      STEPPER_PIN_B2, STEPPER_PIN_B3,
                      STEPPER_PIN_B4);            // Stepper motor B (lift)
Encoder encoderA(ENCODER_PIN_A1, ENCODER_PIN_A2); // Encoder A
Encoder encoderB(ENCODER_PIN_B1, ENCODER_PIN_B2); // Encoder B
SystemStateHandler systemStateHandler =
    SystemStateHandler();                             // System state handler
MovingAverageSensor lineSensorA1(LINE_SENSOR_PIN_A1); // Line sensor A1
MovingAverageSensor lineSensorA2(LINE_SENSOR_PIN_A2); // Line sensor A2
MovingAverageSensor lineSensorA3(LINE_SENSOR_PIN_A3); // Line sensor A3
MovingAverageSensor lineSensorB1(LINE_SENSOR_PIN_B1); // Line sensor B1
MovingAverageSensor lineSensorB2(LINE_SENSOR_PIN_B2); // Line sensor B2
MovingAverageSensor lineSensorB3(LINE_SENSOR_PIN_B3); // Line sensor B3

int stateflowIndex = 0;
int previousStateflowIndex = 0;
int lastError_LINE = 0;
int lastError_ENCODER = 0;
int integral_ENCODER = 0;

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
void handleFollowLine(int mode = REGULAR);
void handleAvoidObstacle();
void initializePins();
void initializeSerialPort();
void attachServoMotors();
void zeroEncoders();
void turnLED(LED_STATE state);
BUTTON_STATE getBUTTON_STATE();
void rotateStepperAdeg(int degrees);
void rotateStepperBdeg(int degrees);
void logError(const char *message);
void setStepperMotorSpeedsToMax();
void servosOff();
void rotateStepperAsteps(int steps);
void rotateStepperBsteps(int steps);
void handleCalibrate();
void handlePIDEncoderDrive(int baseSpeed = 255);
void handleFourBar(int direction);
int combineLineResult(int avg1, int avg2, int avg3);
int determineError(int lineSensorValue);
void resetPIDMemory();
void resetEncoderPID();

// ===== MAIN SETUP =====
void setup() {
  Serial.println("Starting...");

  delay(1000);

  initializePins();
  initializeSerialPort();
  attachServoMotors();
  setStepperMotorSpeedsToMax();
  zeroEncoders();

  systemStateHandler.changeState(SystemState::IR_IDLE);
}

// ===== MAIN LOOP =====
void loop() {
  // On state change.
  if (stateflowIndex != previousStateflowIndex) {
    previousStateflowIndex = stateflowIndex;
    resetPIDMemory();
    delay(500);
  }

  // Order of operations
  switch (stateflowIndex) {
  case 0:
    systemStateHandler.changeState(SystemState::IDLE);
    break;
  case 1:
    systemStateHandler.changeState(SystemState::LINE_FOLLOW_PICKUP);
    break;
  case 2:
    systemStateHandler.changeState(SystemState::AVOID_OBSTACLE);
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
  case SystemState::PID_ENCODER_DRIVE:
    // Code for PID control of encoder drive
    handlePIDEncoderDrive();
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
  case SystemState::AVOID_OBSTACLE:
    handleAvoidObstacle();
    break;
  default:
    logError("Invalid state");
    break;
  }
}

// ===== OTHER FUNCTIONS =====
void handleTest() {
  // Code for testing

  // read the state of the encoder value:
  /*long encoderValueA = encoderA.read();
  long encoderValueB = encoderB.read();

  Serial.print("Encoder Value: ");
  Serial.print(encoderValueA);
  Serial.print(", ");
  Serial.println(encoderValueB);*/

  static unsigned long myTimerStart = millis();
  Serial.print("Time: ");
  Serial.print(millis() - myTimerStart);
  Serial.print(", ");

  if (getBUTTON_STATE() == PRESSED) {
    turnLED(ON);
    logError("Button pressed");
  } else {
    turnLED(OFF);

    if (millis() - myTimerStart < 1000) {
      Serial.print("Motor A Forward");
      // Test motor A (right) FORWARD
      motorDriver.motorAForward();
      motorDriver.motorBStop();
    } else if (millis() - myTimerStart < 2000) {
      Serial.print("Motor A Reverse");
      // Test motor A (right) REVERSE
      motorDriver.motorAReverse();
      motorDriver.motorBStop();
    } else if (millis() - myTimerStart < 3000) {
      Serial.print("Motor B Forward");
      // Test motor B (left) FORWARD
      motorDriver.motorBForward();
      motorDriver.motorAStop();
    } else if (millis() - myTimerStart < 4000) {
      Serial.print("Motor B Reverse");
      // Test motor B (left) REVERSE
      motorDriver.motorBReverse();
      motorDriver.motorAStop();
    } else if (millis() - myTimerStart < 9000) {
      Serial.print("Stepper A Forward");
      // Test stepper A (four-bar) FORWARD
      servosOff();
      rotateStepperAsteps(1);
    } else if (millis() - myTimerStart < 14000) {
      Serial.print("Stepper A Reverse");
      // Test stepper A (four-bar) REVERSE
      servosOff();
      rotateStepperAsteps(-1);
    } else if (millis() - myTimerStart < 16000) {
      Serial.print("Stepper B Forward");
      // Test stepper B (lift) FORWARD
      servosOff();
      rotateStepperBsteps(1);
    } else if (millis() - myTimerStart < 18000) {
      Serial.print("Stepper B Reverse");
      // Test stepper B (lift) REVERSE
      servosOff();
      rotateStepperBsteps(-1);
    } else {
      Serial.print("Resetting...");
      delay(5000);
      myTimerStart = millis();
    }
  }
  Serial.println();
}

void handleIdle() {
  servosOff();
  turnLED(ON);
  if (getBUTTON_STATE() == PRESSED) {
    turnLED(OFF);
    stateflowIndex++;
  }
}

void handleCalibrate(int componentCode) {
  switch (componentCode) {
  case STEPPER_A:
    if (getBUTTON_STATE() == PRESSED) {
      // Hold down button until four-bar crank is in lowest position.
      turnLED(ON);
      rotateStepperAsteps(1);
      delay(200);
    } else {
      turnLED(OFF);
    }
    break;
  case STEPPER_B:
    if (getBUTTON_STATE() == PRESSED) {
      // Hold down button until lift is in lowest position.
      turnLED(ON);
      rotateStepperBsteps(1);
      delay(200);
    } else {
      turnLED(OFF);
    }
    break;
  case SERVO_A:
    if (getBUTTON_STATE() == PRESSED) {
      motorDriver.motorAForward(64); // 25% speed
      turnLED(ON);
    } else {
      servosOff();
      turnLED(OFF);
    }
    break;
  case SERVO_B:
    if (getBUTTON_STATE() == PRESSED) {
      motorDriver.motorBForward(64); // 25% speed
      turnLED(ON);
    } else {
      servosOff();
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
  lineSensorA1.read();
  lineSensorA2.read();
  lineSensorA3.read();
  lineSensorB1.read();
  lineSensorB2.read();
  lineSensorB3.read();

  // Calculate the averages
  int avgA1 = lineSensorA1.average();
  int avgA2 = lineSensorA2.average();
  int avgA3 = lineSensorA3.average();
  int avgB1 = lineSensorB1.average();
  int avgB2 = lineSensorB2.average();
  int avgB3 = lineSensorB3.average();

  Serial.print("Line sensor: ");
  Serial.print(avgA1);
  Serial.print(", ");
  Serial.print(avgA2);
  Serial.print(", ");
  Serial.print(avgA3);
  Serial.print("; ");
  Serial.print(avgB1);
  Serial.print(", ");
  Serial.print(avgB2);
  Serial.print(", ");
  Serial.println(avgB3);

  servosOff();
}

void handlePIDEncoderDrive(int baseSpeed = 255) {
  // PID parameters
  float Kp = ENCODER_DRIVE_KP;
  float Ki = ENCODER_DRIVE_KI;
  float Kd = ENCODER_DRIVE_KD;

  // Read encoder values
  long encoderValueA = encoderA.read();
  long encoderValueB = encoderB.read();

  // Calculate error
  long error = encoderValueA - encoderValueB;

  // PID control
  integral_ENCODER += error;
  float derivative = error - lastError_ENCODER;
  float output = Kp * error + Ki * integral_ENCODER + Kd * derivative;

  // Adjust motor speeds
  int motorSpeedA = constrain(baseSpeed - output, 0, 255);
  int motorSpeedB = constrain(baseSpeed + output, 0, 255);

  motorDriver.motorAForward(motorSpeedA);
  motorDriver.motorBForward(motorSpeedB);

  // Debugging output
  Serial.print("Left: ");
  Serial.print(encoderValueA);
  Serial.print(" Right: ");
  Serial.print(encoderValueB);
  Serial.print(" Error: ");
  Serial.println(error);

  // Update previous error
  lastError_ENCODER = error;

  // Short delay to avoid overwhelming the microcontroller
  delay(100);
}

void handleFollowLine(int mode = REGULAR) {
  bool isSensorBOn = false;

  // Determine if sensor B should be on
  switch (mode) {
  case PICKUP:
    isSensorBOn = true;
    break;
  case REGULAR:
    isSensorBOn = false;
    break;
  case DROPOFF:
    isSensorBOn = true;
    break;
  default:
    logError("Invalid mode");
    break;
  }

  // Read the sensor values
  lineSensorA1.read();
  lineSensorA2.read();
  lineSensorA3.read();
  if (isSensorBOn) {
    lineSensorB1.read();
    lineSensorB2.read();
    lineSensorB3.read();
  }

  // Calculate the line result
  int resultA = combineLineResult(
      lineSensorA1.average(), lineSensorA2.average(), lineSensorA3.average());

  if (isSensorBOn) {
    int resultB = combineLineResult(
        lineSensorB1.average(), lineSensorB2.average(), lineSensorB3.average());
  }

  switch (mode) {
  case PICKUP:
    // Code for line following in pickup mode
    break;
  case REGULAR:
    // Code for regular line following

    float Kp = LINE_FOLLOW_REGULAR_KP; // Proportional gain
    float Kd = LINE_FOLLOW_REGULAR_KD; // Derivative gain

    int error = determineError(resultA);

    int P = error;
    int D = error - lastError_LINE;
    int output = Kp * P + Kd * D;

    int baseSpeed = 150; // Adjust this value as needed
    int leftMotorSpeed = baseSpeed + output;
    int rightMotorSpeed = baseSpeed - output;

    // Ensure motor speeds are within valid range (e.g., 0 to 255 for PWM
    // control)
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

    if (rightMotorSpeed == leftMotorSpeed) {
      handlePIDEncoderDrive(rightMotorSpeed);
    } else {
      resetEncoderPID();
      motorDriver.motorAForward(rightMotorSpeed);
      motorDriver.motorBForward(leftMotorSpeed);
    }

    lastError_LINE = error;
    break;
  case DROPOFF:
    // Code for line following in dropoff mode
    break;
  default:
    logError("Invalid mode");
    break;
  }
}

void handleFourBar(int direction) {
  if (direction == LOAD) {
    // Load the four-bar mechanism
    rotateStepperAdeg(360);
  } else if (direction == UNLOAD) {
    // Unload the four-bar mechanism
    rotateStepperAdeg(-360);
  } else {
    logError("Invalid direction");
  }
}

void handleAvoidObstacle() {
  /*// Assume we need to avoid obstacles for a minimum of 3 seconds
  if (millis() - lastStateChangeTime > 3000) {  // Stay in AVOID_OBSTACLE for at
  least 3000 ms if (obstacleCleared) { changeState(FOLLOW_LINE);
    }
  }*/
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

void attachServoMotors() {
  motorDriver.attachMotorA(SERVO_PIN_A1, SERVO_PIN_A2);
  motorDriver.attachMotorB(SERVO_PIN_B1, SERVO_PIN_B2);
}

void zeroEncoders() {
  encoderA.write(0);
  encoderB.write(0);
}

void turnLED(LED_STATE state) { digitalWrite(LED_PIN, state); }

BUTTON_STATE getBUTTON_STATE() {
  return (!digitalRead(BUTTON_PIN)) == HIGH ? PRESSED : UNPRESSED;
}

void rotateStepperAdeg(int degrees) {
  stepperMotorA.step(degrees / 360.0 * STEPPER_A_STEPS_PER_REVOLUTION);
}

void rotateStepperBdeg(int degrees) {
  stepperMotorB.step(degrees / 360.0 * STEPPER_B_STEPS_PER_REVOLUTION);
}

void rotateStepperAsteps(int steps) { stepperMotorA.step(steps); }

void rotateStepperBsteps(int steps) { stepperMotorB.step(steps); }

void logError(const char *message) {
  systemStateHandler.changeState(SystemState::IDLE);
  turnLED(ON);
  servosOff();
  Serial.println(message);
  while (true)
    ; // Stop the program
}

void setStepperMotorSpeedsToMax() {
  stepperMotorA.setSpeed(STEPPER_A_MAX_SPEED);
  stepperMotorB.setSpeed(STEPPER_B_MAX_SPEED);
}

void servosOff() {
  motorDriver.motorAStop();
  motorDriver.motorBStop();
}

int combineLineResult(int avg1, int avg2, int avg3) {
  // int threshold = LINE_SENSOR_A_THRESHOLD;

  // CONVENTION: 1 = black ON-TARGET, 0 = white OFF-TARGET
  int lineSensorValueA1 = (avg1 < LINE_SENSOR_A_THRESHOLD) ? 1 : 0;
  int lineSensorValueA2 = (avg2 < LINE_SENSOR_A_THRESHOLD) ? 1 : 0;
  int lineSensorValueA3 = (avg3 < LINE_SENSOR_A_THRESHOLD) ? 1 : 0;

  // COMBINE values into one variable (e.g. 001, 000, 111, 101, etc)
  int lineSensorValue =
      (lineSensorValueA1 << 2) | (lineSensorValueA2 << 1) | lineSensorValueA3;

  return lineSensorValue;
}

int determineError(int lineSensorValue) {
  // Determine the error based on the line sensor value
  int error = 0;

  switch (lineSensorValue) {
  case 0b000:
    // Robot is off the line, keep last known direction
    error = 0;
    break;
  case 0b001:
    // Robot needs to turn left
    error = +2;
    break;
  case 0b010:
    // Robot is centered
    error = 0;
    break;
  case 0b011:
    // Robot slightly off center to the right
    error = +1;
    break;
  case 0b100:
    // Robot needs to turn right
    error = -2;
    break;
  case 0b110:
    // Robot slightly off center to the left
    error = -1;
    break;
  case 0b101:
    // Robot is centered
    error = 0;
    break;
  default:
    // Robot is centered
    error = 0;
    break;
  }

  return error;
}

void resetPIDMemory() {
  lastError_LINE = 0;
  resetEncoderPID();
}

void resetEncoderPID() {
  lastError_ENCODER = 0;
  integral_ENCODER = 0;
}