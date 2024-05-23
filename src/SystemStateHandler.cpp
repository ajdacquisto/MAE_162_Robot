#include "SystemStateHandler.h"
#include <Arduino.h> // Add missing include directive

SystemStateHandler::SystemStateHandler() {
    currentState = SystemState::IDLE;
}

void SystemStateHandler::changeState(SystemState::State newState, unsigned long duration) {
    currentState = newState;
    lastStateChangeTime = millis();
    stateDuration = duration;
}

SystemState::State SystemStateHandler::getCurrentState() {
    return currentState;
}

unsigned long SystemStateHandler::getLastStateChangeTime() {
    return lastStateChangeTime;
}


