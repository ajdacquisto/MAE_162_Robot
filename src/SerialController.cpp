#include "SerialController.h"

SerialController::SerialController() {
  // Constructor implementation
}

SerialController::~SerialController() {
  // Destructor implementation
}

void SerialController::init() {
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial)
    ; // Waits for the Serial port to connect.
  printlnWithTimestamp("Starting...");
}

void SerialController::printBinaryWithLeadingZeros(byte number) {
  for (int i = 5; i >= 0; i--) {
    Serial.print(bitRead(number, i));
  }
}

void SerialController::printWithTimestamp(const char *message) {
  Serial.print("< time: ");
  Serial.print(millis() - lastPrintTime);
  lastPrintTime = millis();
  Serial.print(" > ");
  Serial.print(message);
}

void SerialController::printlnWithTimestamp(const char *message) {
  Serial.print("< time: ");
  Serial.print(millis() - lastPrintTime);
  lastPrintTime = millis();
  Serial.print(" > ");
  Serial.println(message);
}