#include "MotorController.h"

// Implement the member functions of the MotorController class here
MotorController::MotorController()
    : motorDriver(), // Motor driver
      motorDriverYellow(), // Motor driver for yellow motor
      stepperMotorA(STEPPER_A_STEPS_PER_REVOLUTION, STEPPER_PIN_A1,
                    STEPPER_PIN_A2, STEPPER_PIN_A3,
                    STEPPER_PIN_A4), // Stepper motor A (four-bar)
      stepperMotorB(STEPPER_B_STEPS_PER_REVOLUTION, STEPPER_PIN_B1,
                    STEPPER_PIN_B2, STEPPER_PIN_B3,
                    STEPPER_PIN_B4) // Stepper motor B (lift)
{}

MotorController::~MotorController() {
  // Clean up any resources
}

void MotorController::attachServoMotors() {
  motorDriver.attachMotorA(SERVO_PIN_A1, SERVO_PIN_A2);
  motorDriver.attachMotorB(SERVO_PIN_B1, SERVO_PIN_B2);

  motorDriverYellow.attachMotorA(YELLOW_MOTOR_PIN_1, YELLOW_MOTOR_PIN_2);
}

void MotorController::rotateStepperAdeg(int degrees) {
  stepperMotorA.step(degrees / 360.0 * STEPPER_A_STEPS_PER_REVOLUTION);
}

void MotorController::rotateStepperAsteps(int steps) {
  stepperMotorA.step(steps);
}

void MotorController::rotateStepperBdeg(int degrees) {
  stepperMotorB.step(degrees / 360.0 * STEPPER_B_STEPS_PER_REVOLUTION);
}

void MotorController::rotateStepperBsteps(int steps) {
  stepperMotorB.step(steps);
}

void MotorController::servosOff() {
  motorDriver.motorAStop();
  motorDriver.motorBStop();
}

void MotorController::setStepperMotorSpeedsToMax() {
  stepperMotorA.setSpeed(STEPPER_A_MAX_SPEED);
  stepperMotorB.setSpeed(STEPPER_B_MAX_SPEED);
}

bool MotorController::rotateRobot(ROTATE_DIRECTION direction, int interruptSensorVal) {
  int ROTATION_SPEED = 255;

  // Get the middle digit of the interrupt sensor value
  int middleDigit = (interruptSensorVal >> 1) & 0b1;

  if (middleDigit == 0 && hasLeftLineYet == false) {
    hasLeftLineYet = true;
  }

  if (middleDigit == 1 && hasLeftLineYet == true) {
    hasLeftLineYet = false;
    servosOff();
    return true;
  }

  // (positive for clockwise, negative for counterclockwise)
  if (direction == ROTATE_DIRECTION::RIGHT) {
    // CLOCKWISE
    // Right (A) backwards, while Left (B) forwards
    motorDriver.motorAReverse(ROTATION_SPEED);
    motorDriver.motorBForward(ROTATION_SPEED);
  } else {
    // COUNTERCLOCKWISE
    // Right (A) forwards, while Left (B) backwards
    motorDriver.motorAForward(ROTATION_SPEED);
    motorDriver.motorBReverse(ROTATION_SPEED);
  }

  return false;
}

MotorController::ROTATE_DIRECTION MotorController::getDirectionToRotate(int pickupLocation) {
  switch(pickupLocation) {
    case FIRST_ON_LEFT:
    case SECOND_ON_LEFT:
    case THIRD_ON_LEFT:
      return ROTATE_DIRECTION::LEFT;
    case FIRST_ON_RIGHT:
    case SECOND_ON_RIGHT:
    case THIRD_ON_RIGHT:
      return ROTATE_DIRECTION::RIGHT;
    default:
      Serial.println("Invalid pickup location");
      return ROTATE_DIRECTION::RIGHT;
  }
  return ROTATE_DIRECTION::RIGHT;
}

void MotorController::servoDrive(SERVO whichServo, int speed) {
  
  switch(whichServo) {
    case SERVO_A:
      if (speed < 0) {
        motorDriver.motorAReverse(-speed);
      } else {
        motorDriver.motorAForward(speed);
      }
      break;
    case SERVO_B:
      if (speed < 0) {
        motorDriver.motorBReverse(-speed);
      } else {
        motorDriver.motorBForward(speed);
      }
      break;
  }
}