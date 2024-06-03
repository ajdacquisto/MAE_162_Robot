/**
 * @file config.h
 * @brief Configuration file for the robot project.
 * 
 * This file contains various settings and parameters used in the robot project.
 * It includes location settings, look ahead settings, software settings, hardware parameters,
 * input and output pin assignments, and a list of all the input and output devices used in the project.
 */
// config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================
// ===== LOCATION SETTINGS =====
// =============================

// - Locations
enum LOCATION {
  FIRST_ON_LEFT,   // 0
  SECOND_ON_LEFT,  // 1
  THIRD_ON_LEFT,   // 2
  FIRST_ON_RIGHT,  // 3
  SECOND_ON_RIGHT, // 4
  THIRD_ON_RIGHT   // 5
};

// - Pickup and dropoff locations
const int PICKUP_LOCATION_1 = FIRST_ON_LEFT;
const int PICKUP_LOCATION_2 = SECOND_ON_LEFT;
const int DROPOFF_LOCATION = SECOND_ON_RIGHT;

// ===============================
// ===== LOOK AHEAD SETTINGS =====
// ===============================

const int LA_BASE_SPEED = 150;
const int LA_MIN_SPEED = 80;
const int LA_UPPER_CONSTRAINT = 80;
const int LA_LOWER_CONSTRAINT = -180;

const float LA_KP = 30.0;
const float LA_KI = 0.0;
const float LA_KD = 5.0;

// =============================
// ===== SOFTWARE SETTINGS =====
// =============================

const int LINE_SENSOR_THRESHOLD = 860;
// - Line Sensor A (front)
// const int LINE_SENSOR_A_THRESHOLD = 850;

// - Line Sensor B (side)
// const int LINE_SENSOR_B_THRESHOLD = 850;

// - Stepper Motor A (four-bar)
const int STEPPER_A_MAX_SPEED = 1;

// - Stepper Motor B (lift)
const int STEPPER_B_MAX_SPEED = 12;

// - Encoder PID drive parameters
const float ENCODER_DRIVE_KP = 0.05;
const float ENCODER_DRIVE_KI = 0.01;
const float ENCODER_DRIVE_KD = 0.005;

// - Line follow PID parameters
const float LINE_FOLLOW_REGULAR_KP = 50.0;
const float LINE_FOLLOW_REGULAR_KI = 1.0;
const float LINE_FOLLOW_REGULAR_KD = 5.0;

// - Other drive parameters
const int REVERSE_SPEED = 100;
const int BASE_SPEED = 100;
const int CONSTRAINT = 250;
const int LOWER_CONSTRAINT = -150;
const float MIN_SPEED = 120;

// - Stop lineFollow for obstacle avoidance
const long OBSTACLE_DISTANCE_THRESHOLD = 10;
const long NINETY_DEGREE_TURN_DISTANCE = 29;

// ===============================
// ===== HARDWARE PARAMETERS =====
// ===============================

// - Stepper Motor A (four-bar)
const int STEPPER_A_STEPS_PER_REVOLUTION = 200;

// - Stepper Motor B (lift)
const int STEPPER_B_STEPS_PER_REVOLUTION = 200;

// - Encoder
// const int ENCODER_MAX_SPEED = 4663;
// const int ENCODER_MAX_SPEED RIGHT = 6500; //ish
// const int ENCODER_MAX_SPEED LEFT = 6773; //ish
const int ENCODER_MAX_SPEED = 6800;

// =================================
// ===== INPUT PIN ASSIGNMENTS =====
// =================================

// - Button
const int BUTTON_PIN = 7;

// - Encoders
const int ENCODER_PIN_A1 = 18, ENCODER_PIN_A2 = 19;
const int ENCODER_PIN_B1 = 20, ENCODER_PIN_B2 = 21;

// - Line Sensor A (front)
const int LINE_SENSOR_PIN_A1 = A2;
const int LINE_SENSOR_PIN_A2 = A1;
const int LINE_SENSOR_PIN_A3 = A0;

// - Line Sensor B (side)
const int LINE_SENSOR_PIN_B1 = A5;
const int LINE_SENSOR_PIN_B2 = A4;
const int LINE_SENSOR_PIN_B3 = A3;

// - Ultrasonic Sensor
const int ULTRASONIC_TRIG_PIN = 46, ULTRASONIC_ECHO_PIN = 50;

// =========================
// ===== NEW IR SENSOR =====
// =========================

const int NEW_IR_PIN_1 = 22; // left
const int NEW_IR_PIN_2 = 23;
const int NEW_IR_PIN_3 = 24;
const int NEW_IR_PIN_4 = 25;
const int NEW_IR_PIN_5 = 26; // right

// ==================================
// ===== OUTPUT PIN ASSIGNMENTS =====
// ==================================

// - LED
const int LED_PIN = LED_BUILTIN;

// - Servo Motor A (right)
const int SERVO_PIN_A1 = 11, SERVO_PIN_A2 = 12;

// - Servo Motor B (left)
const int SERVO_PIN_B1 = 10, SERVO_PIN_B2 = 9;

// - Stepper Motor A (four-bar)
const int STEPPER_PIN_A1 = 40, STEPPER_PIN_A2 = 41;
const int STEPPER_PIN_A3 = 42, STEPPER_PIN_A4 = 43;

// - Stepper Motor B (lift)
const int STEPPER_PIN_B1 = 30, STEPPER_PIN_B2 = 31;
const int STEPPER_PIN_B3 = 32, STEPPER_PIN_B4 = 33;

// ==============================
// ===== NEW STEPPER DRIVER =====
// ==============================

const int STEPPER_A_DIRECTION_PIN = 38;
const int STEPPER_A_STEP_PIN = 36;

const int STEPPER_B_DIRECTION_PIN = 0;
const int STEPPER_B_STEP_PIN = 0;

// ==================================
// ==================================
// ==================================

#endif // CONFIG_H

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