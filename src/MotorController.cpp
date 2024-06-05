#include "MotorController.h"

// Implement the member functions of the MotorController class here
/**
 * @brief Constructs a MotorController object.
 *
 * This constructor initializes the MotorController object by initializing the
 * motor driver, stepper motor drivers, and stepper motors. It sets up the
 * necessary pins and parameters for controlling the stepper motors.
 */
MotorController::MotorController()
    : motorDriver(), // Motor driver,
                     // stepperDriverA(), stepperDriverB(), // Stepper motor
                     // drivers stepperMotorA(STEPPER_A_STEPS_PER_REVOLUTION,
                     // STEPPER_PIN_A1,
                     //               STEPPER_PIN_A2, STEPPER_PIN_A3,
                     //               STEPPER_PIN_A4), // Stepper motor A
                     //               (four-bar)
                     // stepperMotorB(STEPPER_B_STEPS_PER_REVOLUTION,
                     // STEPPER_PIN_B1,
                     //               STEPPER_PIN_B2, STEPPER_PIN_B3,
                     //               STEPPER_PIN_B4), // Stepper motor B (lift)
                     // stepperDriverLift(STEPPER_B_STEPS_PER_REVOLUTION,
                     // STEPPER_B_STEP_PIN,
                     //                   STEPPER_B_DIRECTION_PIN),
      customStepperA(STEPPER_A_DIRECTION_PIN, STEPPER_A_STEP_PIN,
                     STEPPER_A_ENABLE_PIN, STEPPER_A_STEPS_PER_REVOLUTION),
      customStepperB(STEPPER_B_DIRECTION_PIN, STEPPER_B_STEP_PIN,
                     STEPPER_B_ENABLE_PIN, STEPPER_B_STEPS_PER_REVOLUTION) {}

/**
 * @brief Destructor for the MotorController class.
 *
 * This destructor is responsible for cleaning up any resources used by the
 * MotorController object.
 *
 * @note This destructor does not delete the MotorController object itself, as
 * that is the responsibility of the caller.
 */
MotorController::~MotorController() {
  // Clean up any resources
}

/**
 * Initializes the motor controller.
 * This function attaches the motor driver to the motor pins, sets the speed of
 * the stepper motors, configures the stepper drivers, and sets the speed of the
 * lift stepper motor. Additionally, it sets the pinMode for the lift stepper
 * motor's direction pin.
 */
void MotorController::init() {
  motorDriver.attachMotorA(SERVO_PIN_A1, SERVO_PIN_A2);
  motorDriver.attachMotorB(SERVO_PIN_B1, SERVO_PIN_B2);
  servosOff();

  // stepperMotorA.setSpeed(STEPPER_A_MAX_SPEED);
  // stepperMotorB.setSpeed(STEPPER_B_MAX_SPEED);

  // stepperDriverA.begin(STEPPER_A_DIRECTION_PIN, STEPPER_A_STEP_PIN);
  // stepperDriverA.setDirection(DRV8825_CLOCK_WISE);
  // stepperDriverA.setStepsPerRotation(STEPPER_A_STEPS_PER_REVOLUTION);

  // stepperDriverB.begin(STEPPER_B_DIRECTION_PIN, STEPPER_B_STEP_PIN);
  // stepperDriverB.setDirection(DRV8825_CLOCK_WISE);
  // stepperDriverB.setStepsPerRotation(STEPPER_B_STEPS_PER_REVOLUTION);

  // stepperDriverLift.setSpeed(STEPPER_B_MAX_SPEED);
  // pinMode(STEPPER_B_DIRECTION_PIN, OUTPUT);

  pinMode(STEPPER_A_ENABLE_PIN, OUTPUT);
  pinMode(STEPPER_B_ENABLE_PIN, OUTPUT);

  digitalWrite(STEPPER_A_ENABLE_PIN, HIGH); // DISABLE
  digitalWrite(STEPPER_B_ENABLE_PIN, HIGH); // DISABLE
}

/**
 * Drives the specified servo to the desired speed using ramping.
 *
 * @param whichServo The servo to drive (SERVO_A or SERVO_B).
 * @param speed The desired speed for the servo.
 */
void MotorController::servoDrive(SERVO whichServo, int speed) {
  Serial.print("servoDrive: servo");
  if (whichServo == SERVO_A) {
    Serial.print("A ");
  } else {
    Serial.print("B ");
  }
  Serial.println(speed);
  
  if (whichServo == SERVO_B) {
    speed *= -1;
  }
  speed *= -1; // Invert speed for correct direction
  int *lastSpeed;
  void (DRV8833::*forward)(int);
  void (DRV8833::*reverse)(int);

  if (whichServo == SERVO_A) {
    lastSpeed = &lastSpeedA;
    forward = &DRV8833::motorAForward;
    reverse = &DRV8833::motorAReverse;
  } else {
    lastSpeed = &lastSpeedB;
    forward = &DRV8833::motorBForward;
    reverse = &DRV8833::motorBReverse;
  }

  while (*lastSpeed != speed) {
    if (*lastSpeed < speed) {
      *lastSpeed += rampRate;
      if (*lastSpeed > speed) {
        *lastSpeed = speed;
      }
    } else {
      *lastSpeed -= rampRate;
      if (*lastSpeed < speed) {
        *lastSpeed = speed;
      }
    }

    if (*lastSpeed < 0) {
      (motorDriver.*reverse)(-*lastSpeed);
    } else {
      (motorDriver.*forward)(*lastSpeed);
    }

    delay(10); // Small delay for smooth ramping (adjust as needed)
  }
}

/**
 * Turns off the servos by stopping both motor A and motor B.
 */
void MotorController::servosOff() {
  motorDriver.motorAStop();
  motorDriver.motorBStop();
}

/**
 * Drives the specified stepper motor with the given speed and number of steps.
 *
 * @param whichStepper The stepper motor to drive (STEPPER_FOUR_BAR or
 * STEPPER_LIFT).
 * @param speed The speed at which to drive the stepper motor. A negative value
 * indicates reverse direction.
 * @param steps The number of steps to move the stepper motor.
 */
void MotorController::stepperDrive(STEPPER whichStepper, int speed, int steps) {
  bool goBackwards = false;
  if (speed < 0) {
    speed = -speed;
    goBackwards = true;
  }

  switch (whichStepper) {
  case STEPPER_FOUR_BAR:
    if (goBackwards) {
      customStepperA.rotateClockwise(steps, speed);
    } else {
      customStepperA.rotateCounterclockwise(steps, speed);
    }
    break;
  case STEPPER_LIFT:
    if (goBackwards) {
      customStepperB.rotateCounterclockwise(steps, speed);
    } else {
      customStepperB.rotateClockwise(steps, speed);
    }
    break;
  default:
    break;
  }
}

/**
 * Converts degrees to steps for the specified stepper motor.
 *
 * @param degrees The angle in degrees to convert.
 * @param whichStepper The stepper motor to convert the angle for.
 * @return The number of steps corresponding to the given angle.
 */
int MotorController::degToSteps(int degrees, STEPPER whichStepper) {
  switch (whichStepper) {
  case STEPPER_FOUR_BAR:
    return degrees / 360.0 * STEPPER_A_STEPS_PER_REVOLUTION;
  case STEPPER_LIFT:
    return degrees / 360.0 * STEPPER_B_STEPS_PER_REVOLUTION;
  default:
    return 0;
  }
}

/**
 * Rotates the robot in the specified direction until a certain condition is
 * met.
 *
 * @param direction The direction in which to rotate the robot (RIGHT or LEFT).
 * @param interruptSensorVal The value of the interrupt sensor.
 * @return True if the condition is met and the rotation is complete, false
 * otherwise.
 */
bool MotorController::rotateRobot(ROTATE_DIRECTION direction,
                                  int interruptSensorVal) {
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

/**
 * Returns the direction to rotate based on the given pickup location.
 *
 * @param pickupLocation The pickup location to determine the direction to
 * rotate.
 * @return The direction to rotate (LEFT or RIGHT).
 */
MotorController::ROTATE_DIRECTION
MotorController::getDirectionToRotate(int pickupLocation) {
  switch (pickupLocation) {
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

/**
 * Handles the movement of the four-bar mechanism in the specified direction.
 *
 * @param direction The direction in which to move the four-bar mechanism.
 *                  Possible values are LOAD or UNLOAD.
 */
void MotorController::handleFourBar(FOUR_BAR_DIRECTION direction) {
  const int SPEED = 1;
  if (direction == LOAD) {
    // Load the four-bar mechanism
    stepperDrive(STEPPER_FOUR_BAR, SPEED, degToSteps(360, STEPPER_FOUR_BAR));
  } else if (direction == UNLOAD) {
    // Unload the four-bar mechanism
    stepperDrive(STEPPER_FOUR_BAR, -SPEED, degToSteps(360, STEPPER_FOUR_BAR));
  } else {
    Serial.println("Invalid four-bar direction");
  }
}

/**
 * Retrieves the last desired speeds of the left and right motors.
 *
 * @param leftSpeed Reference to the variable where the last desired left speed
 * will be stored.
 * @param rightSpeed Reference to the variable where the last desired right
 * speed will be stored.
 */
void MotorController::getLastDesiredSpeeds(int &leftSpeed, int &rightSpeed) {
  leftSpeed = lastDesiredLeftSpeed;
  rightSpeed = lastDesiredRightSpeed;
}

/**
 * Sets the last desired speeds for the left and right motors.
 *
 * @param leftSpeed The desired speed for the left motor.
 * @param rightSpeed The desired speed for the right motor.
 */
void MotorController::setLastDesiredSpeeds(int leftSpeed, int rightSpeed) {
  lastDesiredLeftSpeed = leftSpeed;
  lastDesiredRightSpeed = rightSpeed;
}

void MotorController::enableStepper(STEPPER whichStepper) {
  switch (whichStepper) {
  case STEPPER_FOUR_BAR:
    digitalWrite(STEPPER_A_ENABLE_PIN, LOW); // ENABLE
    digitalWrite(STEPPER_B_ENABLE_PIN, HIGH); // DISABLE
    break;
  case STEPPER_LIFT:
    digitalWrite(STEPPER_A_ENABLE_PIN, HIGH); // DISABLE
    digitalWrite(STEPPER_B_ENABLE_PIN, LOW); // ENABLE
    break;
  default:
    break;
  }
}

void MotorController::disableSteppers() {
    digitalWrite(STEPPER_A_ENABLE_PIN, HIGH); // DISABLE
    digitalWrite(STEPPER_B_ENABLE_PIN, HIGH); // DISABLE
}