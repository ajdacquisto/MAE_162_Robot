#ifndef CONTROL_GAIN_HANDLER_H
#define CONTROL_GAIN_HANDLER_H

class ControlGainHandler {
public:
  ControlGainHandler();
  ControlGainHandler(float kp, float kd, float ki);
  ~ControlGainHandler();

  void resetError();
  void resetIntegral();
  void reset();

  void setLastError(int error);
  void setIntegral(int integral);

  void incrementIntegral(int error);

  int getLastError();
  int getIntegral();

  void setKp(float kp);
  void setKd(float kd);
  void setKi(float ki);

  float getKp();
  float getKd();
  float getKi();

private:
  int lastError;
  int integral;
  float kp;
  float kd;
  float ki;
};

#endif // CONTROL_GAIN_HANDLER_H