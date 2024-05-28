#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "DRV8833.h"
#include "Stepper.h"
#include "config.h"

class MotorController {
private:
  bool hasLeftLineYet = false;
public:
  DRV8833 motorDriver;
  Stepper stepperMotorA;
  Stepper stepperMotorB;

  enum ROTATE_DIRECTION { LEFT, RIGHT };
  enum SERVO { SERVO_A, SERVO_B };

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
};

#endif // MOTORCONTROLLER_H