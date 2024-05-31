#ifndef CONTROL_GAIN_HANDLER_H
#define CONTROL_GAIN_HANDLER_H

#include <Arduino.h>
class ControlGainHandler {
public:
  ControlGainHandler();
  ControlGainHandler(float kp, float ki, float kd);
  ~ControlGainHandler();

  void resetError();
  void resetIntegral();
  void reset();

  void setLastError(int error);
  void incrementIntegral(int error);

  int getLastError();
  int getIntegral();
  unsigned long getLastIntegralResetTime();

  void setKp(float kp);
  void setKi(float ki);
  void setKd(float kd);

  float getKp();
  float getKi();
  float getKd();

private:
  int lastError;
  int integral;
  float kp;
  float ki;
  float kd;

  unsigned long lastIntegralResetTime;
};
;

#endif // CONTROL_GAIN_HANDLER_H