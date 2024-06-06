#ifndef CONTROL_GAIN_HANDLER_H
#define CONTROL_GAIN_HANDLER_H

#include <Arduino.h>
class ControlGainHandler {
public:
  ControlGainHandler();
  ControlGainHandler(float kp, float ki, float kd);
  ~ControlGainHandler();
  float calculatePID(float error);
  void reset();
private:
  int lastError;
  int integral;
  float kp;
  float ki;
  float kd;
};
;

#endif // CONTROL_GAIN_HANDLER_H