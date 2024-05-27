#ifndef BRANCHHANDLER_H
#define BRANCHHANDLER_H

// Include any necessary frameworks or libraries here

class BranchHandler {
public:
  // Constructor
  BranchHandler() {
    // Initialize any necessary variables or resources
  }

  // Destructor
  ~BranchHandler() {
    // Clean up any allocated resources
  }

private:
  // Member variables
  bool m_isCurrentlyOverBranch;
  int m_targetNum;
  int m_currentLocation;

  // Private getter and setter functions
  void setIsCurrentlyOverBranch(bool isCurrentlyOverBranch);
  void setCurrentLocation(int m_currentLocation);

  // Private helper functions
  bool isMiddleDigitOne(int lineSensorValue);

public:
  // Getters
  bool getIsCurrentlyOverBranch();
  int getTargetNum();
  int getCurrentLocation();

  // Setters
  void setTargetNum(int m_targetNum);

  // Incrementers
  void incrementNumBranchesPassed();
  void incrementTargetNum();

  // Other functions
  void doBranchCheck(int lineSensorValue);
  void reset();
  int getTargetBranchNumFromLocation(int location);
};

#endif // BRANCHHANDLER_H