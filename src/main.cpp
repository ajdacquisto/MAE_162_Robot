#include <Arduino.h>
#include "DRV8833.h"

// ===== SENSORS =====
// - Button
const int buttonPin = 0; // TODO: Define pins
// - Encoder A
const int encoderPinA1 = 0; // TODO: Define pins
const int encoderPinA2 = 0;
// - Encoder B
const int encoderPinB1 = 0;
const int encoderPinB2 = 0;
// - Line Sensor A
const int lineSensorPinA = A0; // TODO: Define pins
// - Line Sensor B
const int lineSensorPinB = A0;
// - Ultrasonic Sensor
const int ultrasonicTrigPin = 0; // TODO: Define pins
const int ultrasonicEchoPin = 0;

// ===== OUTPUTS =====
// - LED
const int ledPin = LED_BUILTIN;
// - Servo Motor A
const int servoPinA1 = 0; // TODO: Define pins
const int servoPinA2 = 0;
// - Servo Motor B
const int servoPinB1 = 0;
const int servoPinB2 = 0;
// - Stepper Motor A
// - Stepper Motor B

// Other Global Variables
int buttonState = 0;   

// Objects
DRV8833 motorDriver = DRV8833();

// Main setup
void setup() {
  // Initialize pins
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(ultrasonicTrigPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);

  // Initialize Serial port
  Serial.begin(9600);
  while (!Serial); // Waits for the Serial port to connect.

  // Attach motors
  motorDriver.attachMotorA(servoPinA1, servoPinA2);
  motorDriver.attachMotorB(servoPinB1, servoPinB2);

}

// Main loop
void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  Serial.println(buttonState);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);

    // Move motors
    motorDriver.motorAForward();
    motorDriver.motorBForward();
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);

    // Stop motors
    motorDriver.motorAStop();
    motorDriver.motorBStop();
  }
}