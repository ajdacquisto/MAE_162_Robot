#include "BranchHandler.h"

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

void BranchHandler::init() {
  m_pickupLocation1 = PICKUP_LOCATION_1;
  m_pickupLocation2 = PICKUP_LOCATION_2;
  m_dropoffLocation = DROPOFF_LOCATION;
}

int BranchHandler::getLocationFromTargetNum(int targetNum) {
  // convention:
  // 1 for pickup location 1
  // 2 for pickup location 2
  // 3 for dropoff location
  switch (targetNum) {
  case 1:
    return m_pickupLocation1;
  case 2:
    return m_pickupLocation2;
  case 3:
    return m_dropoffLocation;
  default:
    return -1;
  }
}

int BranchHandler::getBranchNumFromLocation(int location) {
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

void BranchHandler::setTargetNum(int targetNum) {
  m_targetNum = targetNum;
  m_targetLocation = getLocationFromTargetNum(m_targetNum);
  m_targetBranch = getBranchNumFromLocation(m_targetLocation);
}

int BranchHandler::getTargetLocation() { return m_targetLocation; }

int BranchHandler::getTargetBranch() { return m_targetBranch; }

bool BranchHandler::isAtTargetLocation() {
  if (getIsCurrentlyOverBranch() && 
      getCurrentLocation() == getTargetLocation()) {
    return true;
  } else {
    return false;
  }
}