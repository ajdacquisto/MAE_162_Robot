#include "BranchHandler.h"
#include "config.h"

// Implement the member functions of BranchHandler class here

// Constructor
BranchHandler::BranchHandler() {
  // Initialize any variables or objects here
  m_isCurrentlyOverBranch = false;
  m_currentLocation = 0;
}

// Destructor
BranchHandler::~BranchHandler() {
  // Clean up any dynamically allocated memory or resources here
}

void BranchHandler::incrementNumBranchesPassed() { m_currentLocation++; }

void BranchHandler::reset() {
  m_isCurrentlyOverBranch = false;
  m_currentLocation = 0;
}

bool BranchHandler::isMiddleDigitOne(int lineSensorValue) {
  // Extract the middle digit using bitwise AND and shift
  int middleDigit = (lineSensorValue >> 1) & 0b1;

  // Return true if the middle digit is 1, false otherwise
  return middleDigit == 1;
}

void BranchHandler::setTargetNum(int targetNum) { m_targetNum = targetNum; }

void BranchHandler::incrementTargetNum() { m_targetNum += 1; }

int BranchHandler::getTargetNum() { return m_targetNum; }

void BranchHandler::setIsCurrentlyOverBranch(bool isCurrentlyOverBranch) {
  if (m_isCurrentlyOverBranch == isCurrentlyOverBranch) {
    return;
  } else if (isCurrentlyOverBranch) {
    // Car is encountering the branch
    incrementNumBranchesPassed();
    m_isCurrentlyOverBranch = isCurrentlyOverBranch;
  } else {
    // Car is leaving the branch
    m_isCurrentlyOverBranch = isCurrentlyOverBranch;
  }
}

bool BranchHandler::getIsCurrentlyOverBranch() {
  return m_isCurrentlyOverBranch;
}

void BranchHandler::doBranchCheck(int lineSensorValue) {
  setIsCurrentlyOverBranch(isMiddleDigitOne(lineSensorValue));
}

void BranchHandler::setCurrentLocation(int m_currentLocation) {
  this->m_currentLocation = m_currentLocation;
}

int BranchHandler::getCurrentLocation() { return m_currentLocation; }

int BranchHandler::getTargetBranchNumFromLocation(int location) {
  switch (location) {
  case FIRST_ON_LEFT:
    return 1;
  case SECOND_ON_LEFT:
    return 2;
  case THIRD_ON_LEFT:
    return 3;
  case FIRST_ON_RIGHT:
    return 1;
  case SECOND_ON_RIGHT:
    return 2;
  case THIRD_ON_RIGHT:
    return 3;
  default:
    return -1;
  }
}