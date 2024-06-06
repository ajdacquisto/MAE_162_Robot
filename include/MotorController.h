#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "CustomStepper.h"
#include "DRV8833.h"
#include "config.h"

class MotorController {
private:
  bool hasLeftLineYet = false;
  int lastSpeedA = 0;      // Last commanded speed for motor A
  int lastSpeedB = 0;      // Last commanded speed for motor B
  const int rampRate = 15; // Tunable ramp rate (adjust as needed)

  int lastDesiredLeftSpeed = 0;
  int lastDesiredRightSpeed = 0;

public:
  DRV8833 motorDriverFront;
  DRV8833 motorDriverRear;
  CustomStepper customStepperA;
  CustomStepper customStepperB;

  enum ROTATE_DIRECTION { LEFT, RIGHT };
  enum COMPONENT { SINGLE_SERVO, FOUR_BAR, LIFT, ALL_WHEELS };
  enum SERVO {
    SERVO_FRONT_RIGHT,
    SERVO_FRONT_LEFT,
    SERVO_REAR_RIGHT,
    SERVO_REAR_LEFT
  };
  enum STEPPER { STEPPER_LIFT, STEPPER_FOUR_BAR };
  enum FOUR_BAR_DIRECTION { LOAD, UNLOAD };
  enum LIFT_DIRECTION { DOWN, UP };
  enum MOTOR_DIRECTION { FORWARD, BACKWARD };

  // Constructor
  MotorController();
  ~MotorController();
  void init();

  // Servos
  void servoDrive(SERVO whichServo, int speed);
  void servosOff();

  void raiseLift();
  void lowerLift();

  void raiseLift(int steps);
  void lowerLift(int steps);

  // Steppers
  void stepperDrive(STEPPER whichStepper, int speed, int steps);
  int degToSteps(int degrees, STEPPER whichStepper);

  bool rotateRobot(ROTATE_DIRECTION direction, int interruptSensorVal);
  ROTATE_DIRECTION getDirectionToRotate(int pickupLocation);
  void handleFourBar(FOUR_BAR_DIRECTION direction);

  void getLastDesiredSpeeds(int &leftSpeed, int &rightSpeed);
  void setLastDesiredSpeeds(int leftSpeed, int rightSpeed);

  void enableStepper(STEPPER whichStepper);
  void disableSteppers();
};

#endif // MOTORCONTROLLER_H