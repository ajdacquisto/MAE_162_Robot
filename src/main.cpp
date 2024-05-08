#include <Arduino.h>
#include "DRV8833.h"
#include "Stepper.h"

// ===== SENSORS =====
// - Button
const int BUTTON_PIN = 0; // TODO: Define pins
// - Encoder A
const int ENCODER_PIN_A1 = 0, ENCODER_PIN_A2 = 0; // TODO: Define pins
// - Encoder B
const int ENCODER_PIN_B1 = 0, ENCODER_PIN_B2 = 0; // TODO: Define pins
// - Line Sensor A (front)
const int LINE_SENSOR_PIN_A1 = 0, LINE_SENSOR_PIN_A2 = 0, LINE_SENSOR_PIN_A3 = 0; // TODO: Define pins
// - Line Sensor B (side)
const int LINE_SENSOR_PIN_B = 0; // TODO: Define pins
// - Ultrasonic Sensor
const int ULTRASONIC_TRIG_PIN = 0, ULTRASONIC_ECHO_PIN = 0; // TODO: Define pins

// ===== OUTPUTS =====
// - LED
const int LED_PIN = LED_BUILTIN;
// - Servo Motor A
const int SERVO_PIN_A1 = 0, SERVO_PIN_A2 = 0; // TODO: Define pins
// - Servo Motor B
const int SERVO_PIN_B1 = 0, SERVO_PIN_B2 = 0; // TODO: Define pins
// - Stepper Motor A
const int STEPPER_PIN_A1 = 0, STEPPER_PIN_A2 = 0, STEPPER_PIN_A3 = 0, STEPPER_PIN_A4 = 0; // TODO: Define pins
const int STEPPER_A_STEPS_PER_REVOLUTION = 0;  // TODO: Set this value
// - Stepper Motor B
const int STEPPER_PIN_B1 = 0, STEPPER_PIN_B2 = 0, STEPPER_PIN_B3 = 0, STEPPER_PIN_B4 = 0; // TODO: Define pins
const int STEPPER_B_STEPS_PER_REVOLUTION = 0;  // TODO: Set this value

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

// ===== MAIN SETUP =====
void setup() {
  // Sensors
  pinMode(BUTTON_PIN, INPUT); // Button
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
  // Outputs
  pinMode(LED_PIN, OUTPUT); // LED

  // Initialize Serial port
  Serial.begin(9600);
  while (!Serial); // Waits for the Serial port to connect.

  // Set initial state
  currentState = IDLE;

  // Attach servo motors
  motorDriver.attachMotorA(SERVO_PIN_A1, SERVO_PIN_A2);
  motorDriver.attachMotorB(SERVO_PIN_B1, SERVO_PIN_B2);

  // Set stepper motor speed
  stepperMotorA.setSpeed(60);  // Set the speed to 60 RPMs
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
  buttonState = digitalRead(BUTTON_PIN);

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