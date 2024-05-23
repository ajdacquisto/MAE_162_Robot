// config.h

#include <Arduino.h>

// ===== SENSORS =====
// - Button
const int BUTTON_PIN = 7; // TODO: Define pins
// - Encoder A
const int ENCODER_PIN_A1 = 18, ENCODER_PIN_A2 = 19; // TODO: Define pins
// - Encoder B
const int ENCODER_PIN_B1 = 20, ENCODER_PIN_B2 = 21; // TODO: Define pins
// - Line Sensor A (front)
const int LINE_SENSOR_PIN_A1 = A0, LINE_SENSOR_PIN_A2 = A1, LINE_SENSOR_PIN_A3 = A2; // TODO: Define pins
// - Line Sensor B (side)
//const int LINE_SENSOR_PIN_B = A3; // TODO: Define pins
const int LINE_SENSOR_PIN_B1 = A3, LINE_SENSOR_PIN_B2 = A4, LINE_SENSOR_PIN_B3 = A5; // TODO: Define pins
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
const int STEPPER_PIN_A1 = 40, STEPPER_PIN_A2 = 41, STEPPER_PIN_A3 = 42, STEPPER_PIN_A4 = 43; // TODO: Define pins
const int STEPPER_A_STEPS_PER_REVOLUTION = 200;  // TODO: Set this value
const int STEPPER_A_MAX_SPEED = 30;  // TODO: Set this value
// - Stepper Motor B (lift)
const int STEPPER_PIN_B1 = 30, STEPPER_PIN_B2 = 31, STEPPER_PIN_B3 = 32, STEPPER_PIN_B4 = 33; // TODO: Define pins
const int STEPPER_B_STEPS_PER_REVOLUTION = 200;  // TODO: Set this value
const int STEPPER_B_MAX_SPEED = 9;  // TODO: Set this value