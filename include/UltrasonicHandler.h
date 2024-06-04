#ifndef ULTRASONICHANDLER_H
#define ULTRASONICHANDLER_H

#include "config.h"
#include <Arduino.h>
#include <HCSR04.h>

class UltrasonicHandler {
public:
  UltrasonicHandler();
  ~UltrasonicHandler();

  // Add your member functions here

  void init();

  long getDist();
  long getOldDist();
  long getMemory();

  unsigned long getNewDistTime();
  unsigned long getOldDistTime();

  bool isObstacle(long threshold);

  void setMemory(long setVal);

private:
  // Add your member variables here
  unsigned long oldTime_ = 0;
  unsigned long newTime_ = 0;
  long newDist_ = 0;
  long oldDist_ = 0;
  long memory = 0;
  HCSR04 hc; // Ultrasonic sensor

  // Add your private member functions here
  void setDist(long setVal);
  void setOldDist(long setVal);

  void setNewDistTime(unsigned long setVal);
  void setOldDistTime(unsigned long setVal);
};

#endif // ULTRASONICHANDLER_H