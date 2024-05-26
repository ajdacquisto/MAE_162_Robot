#ifndef SYSTEMSTATEHANDLER_H
#define SYSTEMSTATEHANDLER_H

// Define states
namespace SystemState {
enum State {
  TEST,
  IDLE,
  FOLLOW_LINE,
  AVOID_OBSTACLE,
  IR_IDLE,
  PID_ENCODER_DRIVE,
  CALIBRATE,
  FOUR_BAR_LOAD,
  FOUR_BAR_UNLOAD,
  LINE_FOLLOW_PICKUP,
  LINE_FOLLOW_DROPOFF
};
}

class SystemStateHandler {
private:
  SystemState::State currentState;
  unsigned long lastStateChangeTime;
  unsigned long stateDuration;

public:
  SystemStateHandler(); // constructor declaration
  void changeState(SystemState::State newState,
                   unsigned long duration = 0); // method declaration
  SystemState::State getCurrentState();         // method declaration
  unsigned long
  getLastStateChangeTime(); // Add other methods to get the state if necessary

  // Add other methods to get the state if necessary
};

#endif // SYSTEMSTATEHANDLER_H