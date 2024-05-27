#ifndef SYSTEMSTATEHANDLER_H
#define SYSTEMSTATEHANDLER_H

// Define states
namespace SystemState {
enum State {
  TEST,
  IDLE,
  FOLLOW_LINE,
  IR_IDLE,
  ULTRASONIC_IDLE,
  PID_ENCODER_DRIVE,
  CALIBRATE,
  FOUR_BAR_LOAD,
  FOUR_BAR_UNLOAD,
  LINE_FOLLOW_PICKUP,
  LINE_FOLLOW_DROPOFF,
  ROTATE_LEFT,
  ROTATE_RIGHT
};
}

class SystemStateHandler {
private:
  SystemState::State currentState;
  unsigned long lastStateChangeTime;
  unsigned long stateDuration;
  int stateflowIndex;
  int previousStateflowIndex;

public:
  SystemStateHandler(); // constructor declaration
  void changeState(SystemState::State newState,
                   unsigned long duration = 0); // method declaration
  SystemState::State getCurrentState();         // method declaration
  unsigned long getLastStateChangeTime();
  int getStateFlowIndex();
  void advanceStateFlowIndex();
  bool isNewStateFlowIndex();
};

#endif // SYSTEMSTATEHANDLER_H