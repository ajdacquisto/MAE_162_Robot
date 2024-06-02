#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "DRV8833.h"
#include "Stepper.h"
#include "config.h"

class MotorController {
private:
  bool hasLeftLineYet = false;
  int lastSpeedA = 0; // Last commanded speed for motor A
  int lastSpeedB = 0; // Last commanded speed for motor B
  const int rampRate = 10; // Tunable ramp rate (adjust as needed)

public:
  DRV8833 motorDriver;
  Stepper stepperMotorA;
  Stepper stepperMotorB;

  enum ROTATE_DIRECTION { LEFT, RIGHT };
  enum COMPONENT {
    RIGHT_WHEEL,
    LEFT_WHEEL,
    FOUR_BAR,
    LIFT,
    BOTH_WHEELS
  };
  enum SERVO { SERVO_A, SERVO_B };

  enum FOUR_BAR_DIRECTION { LOAD, UNLOAD };

  enum LIFT_DIRECTION { DOWN, UP };

  enum MOTOR_DIRECTION { FORWARD, BACKWARD };

  // Constructor
  MotorController();
  ~MotorController();
  void attachServoMotors();
  void rotateStepperAdeg(int degrees);
  void rotateStepperAsteps(int steps);
  void rotateStepperBdeg(int degrees);
  void rotateStepperBsteps(int steps);
  void servosOff();
  void setStepperMotorSpeedsToMax();
  bool rotateRobot(ROTATE_DIRECTION direction, int interruptSensorVal);
  ROTATE_DIRECTION getDirectionToRotate(int pickupLocation);
  void servoDrive(SERVO whichServo, int speed);
  void handleFourBar(FOUR_BAR_DIRECTION direction);
  void init();
};

#endif // MOTORCONTROLLER_H