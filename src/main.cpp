// This is the main Arduino library, which provides the core functionality for Arduino.
#include <Arduino.h>
// This library provides an interface for controlling motors using the DRV8833 motor driver.
#include "DRV8833.h"
// This library provides an interface for controlling stepper motors.
#include "Stepper.h"
// This is the configuration file for this project. It contains definitions for various constants and settings.
#include "config.h"

// Other Global Variables
int buttonState = 0;   

// Objects
DRV8833 motorDriver = DRV8833();
Stepper stepperMotorA(STEPPER_A_STEPS_PER_REVOLUTION, STEPPER_PIN_A1, STEPPER_PIN_A2, STEPPER_PIN_A3, STEPPER_PIN_A4);
Stepper stepperMotorB(STEPPER_B_STEPS_PER_REVOLUTION, STEPPER_PIN_B1, STEPPER_PIN_B2, STEPPER_PIN_B3, STEPPER_PIN_B4);

// ===== STATES =====
// Define states
enum State {
  TEST,
  IDLE,
  FOLLOW_LINE,
  AVOID_OBSTACLE
};

// Current state variable
State currentState = IDLE;

// ===== TIME CONTROL =====
unsigned long lastStateChangeTime = 0;  // Tracks the last time the state changed
unsigned long stateDuration = 0;        // Duration to stay in a particular state, if necessary

// ===== FUNCTION PROTOTYPES =====
void handleTest();
void handleIdle();
void handleFollowLine();
void handleAvoidObstacle();
void initializePins();
void initializeSerialPort();
void attachServoMotors();

// ===== MAIN SETUP =====
void setup() {
  initializePins();
  initializeSerialPort();

  currentState = IDLE; // Set initial state

  attachServoMotors();

  // Set stepper motor speed
  stepperMotorA.setSpeed(60);  // Set the speed to 60 RPM
  stepperMotorB.setSpeed(60);

}

// ===== MAIN LOOP =====
void loop() {
  switch (currentState) {
    case TEST:
      handleTest();
      break;
    case IDLE:
      handleIdle();
      break;
    case FOLLOW_LINE:
      handleFollowLine();
      break;
    case AVOID_OBSTACLE:
      handleAvoidObstacle();
      break;
  }
}

// ===== STATE HANDLERS =====
void changeState(State newState, unsigned long duration = 0) {
  currentState = newState;
  lastStateChangeTime = millis();
  stateDuration = duration;  // If you want a state to last a minimum amount of time, use this
}

void handleTest() {
  // Code for testing

  // read the state of the pushbutton value:
  bool buttonPressed = !digitalRead(BUTTON_PIN);  // Invert the reading here

  Serial.println(buttonState);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(LED_PIN, HIGH);

    // Move motors
    motorDriver.motorAForward();
    motorDriver.motorBForward();

    // Rotate stepper motor
    stepperMotorA.step(STEPPER_A_STEPS_PER_REVOLUTION);
    delay(500);

  } else {
    // turn LED off:
    digitalWrite(LED_PIN, LOW);

    // Stop motors
    motorDriver.motorAStop();
    motorDriver.motorBStop();
  }
}

void handleIdle() {
  /*if (conditionToStartFollowingLine) {
    changeState(FOLLOW_LINE, 5000);  // Stay in FOLLOW_LINE for at least 5000 ms
  }*/
}


void handleFollowLine() {
  /*if (millis() - lastStateChangeTime > stateDuration) {  // Ensure at least stateDuration has passed
    if (lineLost) {
      changeState(IDLE);
    } else if (obstacleDetected) {
      changeState(AVOID_OBSTACLE);
    }
  }*/
}

void handleAvoidObstacle() {
  /*// Assume we need to avoid obstacles for a minimum of 3 seconds
  if (millis() - lastStateChangeTime > 3000) {  // Stay in AVOID_OBSTACLE for at least 3000 ms
    if (obstacleCleared) {
      changeState(FOLLOW_LINE);
    }
  }*/
}

// ===== HELPER FUNCTIONS =====
void initializePins() {
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Button
  pinMode(ENCODER_PIN_A1, INPUT_PULLUP); // Encoder A
  pinMode(ENCODER_PIN_A2, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B1, INPUT_PULLUP); // Encoder B
  pinMode(ENCODER_PIN_B2, INPUT_PULLUP);
  pinMode(LINE_SENSOR_PIN_A1, INPUT); // Line Sensor A
  pinMode(LINE_SENSOR_PIN_A2, INPUT);
  pinMode(LINE_SENSOR_PIN_A3, INPUT);
  pinMode(LINE_SENSOR_PIN_B, INPUT); // Line Sensor B
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); // Ultrasonic Sensor
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT); // LED
}

void initializeSerialPort() {
  Serial.begin(9600); // Initialize Serial port
  while (!Serial); // Waits for the Serial port to connect.
}

void attachServoMotors() {
  motorDriver.attachMotorA(SERVO_PIN_A1, SERVO_PIN_A2);
  motorDriver.attachMotorB(SERVO_PIN_B1, SERVO_PIN_B2);
}