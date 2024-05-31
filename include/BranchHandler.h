#ifndef BRANCHHANDLER_H
#define BRANCHHANDLER_H

// Include any necessary frameworks or libraries here
#include "config.h"

class BranchHandler {
public:
  // Constructor
  BranchHandler();

  // Destructor
  ~BranchHandler();

  enum LOCATION_TYPE {
    PICKUP_LOCATION_1,
    PICKUP_LOCATION_2,
    DROPOFF_LOCATION
  };

private:
  // Member variables
  bool m_isCurrentlyOverBranch;

  // All these are locations as defined in config.h
  int m_currentLocation;
  int m_pickupLocation1 = 0;
  int m_pickupLocation2 = 0;
  int m_dropoffLocation = 0;

  // 1, 2, or 3:
  // (1) Pickup location 1
  // (2) Pickup location 2
  // (3) Dropoff location
  int m_targetNum;

  // Location. Location of the target.
  // targetNum -> targetLocation -> targetBranch
  int m_targetLocation = 0;

  // 1, 2, or 3:
  // (1) First branch (left or right)
  // (2) Second branch (left or right)
  // (3) Third branch (left or right)
  int m_targetBranch = 0;

  // Private getter and setter functions
  void setIsCurrentlyOverBranch(bool isCurrentlyOverBranch);
  void setCurrentLocation(int m_currentLocation);

  // Private helper functions
  bool isMiddleDigitOne(int lineSensorValue);

  int getLocationFromTargetNum(int targetNum);
  int getBranchNumFromLocation(int location);

public:
  // Getters
  bool getIsCurrentlyOverBranch();
  int getCurrentLocation();
  int getTargetNum();
  int getTargetLocation();
  int getTargetBranch();

  // Setters
  void setTargetNum(int m_targetNum);

  // Incrementers
  void incrementNumBranchesPassed();
  void incrementTargetNum();

  // Other functions
  void doBranchCheck(int lineSensorValue);
  void reset();
  //int getTargetBranchNumFromLocation(int location);

  void init();

  bool isAtTargetLocation();
};

#endif // BRANCHHANDLER_H