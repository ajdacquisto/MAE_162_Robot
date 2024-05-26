#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "DRV8833.h"
#include "Stepper.h"
#include "config.h"

class MotorController {
private:
  Stepper stepperMotorA;
  Stepper stepperMotorB;

public:
  DRV8833 motorDriver;
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
};

#endif // MOTORCONTROLLER_H