#ifndef SYSTEMSTATEHANDLER_H
#define SYSTEMSTATEHANDLER_H

// Define states
namespace SystemState {
enum State {
  TEST,
  IDLE,
  LINE_FOLLOW_REGULAR,
  IR_IDLE,
  ULTRASONIC_IDLE,
  CALIBRATE,
  FOUR_BAR_LOAD,
  FOUR_BAR_UNLOAD,
  LINE_FOLLOW_PICKUP,
  LINE_FOLLOW_DROPOFF,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  ULTRASONIC_APPROACH,
  ULTRASONIC_REVERSE,
  LIFT_LOWER,
  LIFT_RAISE
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
  void setStateFlowIndex(int setVal);

  void init(SystemState::State state);
};

#endif // SYSTEMSTATEHANDLER_H