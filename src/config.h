// config.h

#include <Arduino.h>

// ===== SETTINGS =====
enum LOCATION {
  FIRST_ON_LEFT,   // 0
  SECOND_ON_LEFT,  // 1
  THIRD_ON_LEFT,   // 2
  FIRST_ON_RIGHT,  // 3
  SECOND_ON_RIGHT, // 4
  THIRD_ON_RIGHT   // 5
};

const int PICKUP_LOCATION_1 = FIRST_ON_LEFT;
const int PICKUP_LOCATION_2 = SECOND_ON_LEFT;
const int DROPOFF_LOCATION = SECOND_ON_RIGHT;

// ===== SENSORS =====

// - Button
const int BUTTON_PIN = 7;

// - Encoder A
const int ENCODER_PIN_A1 = 18, ENCODER_PIN_A2 = 19;

// - Encoder B
const int ENCODER_PIN_B1 = 20, ENCODER_PIN_B2 = 21;

// - Line Sensor A (front)
const int LINE_SENSOR_PIN_A1 = A0, LINE_SENSOR_PIN_A2 = A1,
          LINE_SENSOR_PIN_A3 = A2;
const int LINE_SENSOR_A_THRESHOLD = 500;

// - Line Sensor B (side)
const int LINE_SENSOR_PIN_B1 = A3, LINE_SENSOR_PIN_B2 = A4,
          LINE_SENSOR_PIN_B3 = A5;
const int LINE_SENSOR_B_THRESHOLD = 500;

// - Ultrasonic Sensor
const int ULTRASONIC_TRIG_PIN = 0, ULTRASONIC_ECHO_PIN = 0; // TODO: Define pins

// ===== OUTPUTS =====

// - LED
const int LED_PIN = LED_BUILTIN;

// - Servo Motor A (right)
const int SERVO_PIN_A1 = 11, SERVO_PIN_A2 = 12; // TODO: Define pins

// - Servo Motor B (left)
const int SERVO_PIN_B1 = 9, SERVO_PIN_B2 = 10; // TODO: Define pins

// - Stepper Motor A (four-bar)
const int STEPPER_PIN_A1 = 40, STEPPER_PIN_A2 = 41, STEPPER_PIN_A3 = 42,
          STEPPER_PIN_A4 = 43;                  // TODO: Define pins
const int STEPPER_A_STEPS_PER_REVOLUTION = 200; // TODO: Set this value
const int STEPPER_A_MAX_SPEED = 30;             // TODO: Set this value

// - Stepper Motor B (lift)
const int STEPPER_PIN_B1 = 30, STEPPER_PIN_B2 = 31, STEPPER_PIN_B3 = 32,
          STEPPER_PIN_B4 = 33;                  // TODO: Define pins
const int STEPPER_B_STEPS_PER_REVOLUTION = 200; // TODO: Set this value
const int STEPPER_B_MAX_SPEED = 9;              // TODO: Set this value

// ===== CONTROL GAINS =====
const float ENCODER_DRIVE_KP = 1.0;
const float ENCODER_DRIVE_KD = 0.0;
const float ENCODER_DRIVE_KI = 0.0;

const float LINE_FOLLOW_REGULAR_KP = 0.5;
const float LINE_FOLLOW_REGULAR_KD = 0.1;

// All IO list:

// INP button [1 pin]
// INP encoder A (right wheel) [2 pins]
// INP encoder B (left wheel) [2 pins]
// INP line sensor A (front) [3 pins]
// INP line sensor B (side) [3 pins]
// INP ultrasonic sensor [2 pins]

// OUT led [1 pin]
// OUT servo A (right) [2 pins]
// OUT servo B (left) [2 pins]
// OUT stepper A (four-bar) [4 pins]
// OUT stepper B (lift) [4 pins]
