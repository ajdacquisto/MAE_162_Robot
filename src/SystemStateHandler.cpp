#include "SystemStateHandler.h"
#include <Arduino.h> // Add missing include directive

SystemStateHandler::SystemStateHandler() {
    currentState = SystemState::TEST;
}

void SystemStateHandler::changeState(SystemState::State newState, unsigned long duration) {
    currentState = newState;
    lastStateChangeTime = millis();
    stateDuration = duration;
}

SystemState::State SystemStateHandler::getCurrentState() {
    return currentState;
}
