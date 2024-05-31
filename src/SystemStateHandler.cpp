#include "SystemStateHandler.h"
#include <Arduino.h> // Add missing include directive

SystemStateHandler::SystemStateHandler() {
    currentState = SystemState::IDLE;
    stateflowIndex = 0;
    previousStateflowIndex = -1;
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

int SystemStateHandler::getStateFlowIndex() {
    return stateflowIndex;
}

void SystemStateHandler::advanceStateFlowIndex() {
    stateflowIndex++;
}

bool SystemStateHandler::isNewStateFlowIndex() {
    if (stateflowIndex != previousStateflowIndex) {
        previousStateflowIndex = stateflowIndex;
        return true;
    } else {
        return false;
    }
}

void SystemStateHandler::init(SystemState::State state) {
    currentState = state;
    stateflowIndex = 0;
    previousStateflowIndex = -1;
    lastStateChangeTime = millis();
    stateDuration = 0;
}