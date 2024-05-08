#include <Arduino.h>
#include "DRV8833.h"

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
// - Stepper Motor B

// Other Global Variables
int buttonState = 0;   

// Objects
DRV8833 motorDriver = DRV8833();

// Main setup
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
  
  // Attach motors
  motorDriver.attachMotorA(SERVO_PIN_A1, SERVO_PIN_A2);
  motorDriver.attachMotorB(SERVO_PIN_B1, SERVO_PIN_B2);

}

// Main loop
void loop() {
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
  } else {
    // turn LED off:
    digitalWrite(LED_PIN, LOW);

    // Stop motors
    motorDriver.motorAStop();
    motorDriver.motorBStop();
  }
}
