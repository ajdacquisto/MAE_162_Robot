#include "UltrasonicHandler.h"

// Constructor
UltrasonicHandler::UltrasonicHandler()
    : hc(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN) {
  // Initialize any variables or objects here
}

// Destructor
UltrasonicHandler::~UltrasonicHandler() {
  // Clean up any resources here
}

void UltrasonicHandler::init() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
}

long UltrasonicHandler::getDist() {
  setOldDist(newDist_);
  long reading = hc.dist();
  setDist(reading);
  setOldDistTime(newTime_);
  setNewDistTime(millis());
  return reading;
}

long UltrasonicHandler::getOldDist() {}

unsigned long UltrasonicHandler::getNewDistTime() {}

unsigned long UltrasonicHandler::getOldDistTime() {}

bool UltrasonicHandler::isObstacle(long threshold) {
  if (millis() - getNewDistTime() < 1000) {
    return false;
  }

  long distance = getDist();
  long prevDist = getOldDist();

  Serial.print("Ultrasonic data: (old) ");
  Serial.print(prevDist);
  Serial.print(", (new) ");
  Serial.println(distance);

  if (distance < threshold && prevDist < threshold) {
    Serial.print(distance);
    Serial.print(" and ");
    Serial.print(prevDist);
    Serial.println(" are both less than ");
    Serial.println(threshold);
    return true;
  } else {
    return false;
  }
}

void UltrasonicHandler::setOldDist(long setVal) { oldDist_ = setVal; }

void UltrasonicHandler::setOldDistTime(unsigned long setVal) {
  oldTime_ = setVal;
}

void UltrasonicHandler::setDist(long setVal) { newDist_ = setVal; }

void UltrasonicHandler::setNewDistTime(unsigned long setVal) {
  newTime_ = setVal;
}

void UltrasonicHandler::setMemory(long setVal) { memory = setVal; }

long UltrasonicHandler::getMemory() { return memory; }