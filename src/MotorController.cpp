#include "MotorController.h"

// Implement the member functions of the MotorController class here
MotorController::MotorController()
    : motorDriver(), // Motor driver
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
