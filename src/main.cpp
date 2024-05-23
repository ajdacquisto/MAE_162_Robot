// This is the main Arduino library, which provides the core functionality for Arduino.
#include <Arduino.h>
// This library provides an interface for controlling motors using the DRV8833 motor driver.
#include "DRV8833.h"
// This library provides an interface for controlling stepper motors.
#include "Stepper.h"
// This is the configuration file for this project. It contains definitions for various constants and settings.
#include "config.h"
// This library provides an interface for reading rotary encoders.
#include "Encoder.h"
// This library provides an interface for managing the state of the system.
#include "SystemStateHandler.h"

#include "MovingAverageSensor.h"

#define BUFFER_SIZE 10

// ===== GLOBAL VARIABLES =====
DRV8833 motorDriver = DRV8833();
Stepper stepperMotorA(STEPPER_A_STEPS_PER_REVOLUTION, STEPPER_PIN_A1, STEPPER_PIN_A2, STEPPER_PIN_A3, STEPPER_PIN_A4);
Stepper stepperMotorB(STEPPER_B_STEPS_PER_REVOLUTION, STEPPER_PIN_B1, STEPPER_PIN_B2, STEPPER_PIN_B3, STEPPER_PIN_B4);
Encoder encoderA(ENCODER_PIN_A1, ENCODER_PIN_A2);
Encoder encoderB(ENCODER_PIN_B1, ENCODER_PIN_B2);
SystemStateHandler systemStateHandler = SystemStateHandler();
MovingAverageSensor lineSensorA1(LINE_SENSOR_PIN_A1);
MovingAverageSensor lineSensorA2(LINE_SENSOR_PIN_A2);
MovingAverageSensor lineSensorA3(LINE_SENSOR_PIN_A3);
MovingAverageSensor lineSensorB(LINE_SENSOR_PIN_B);

// ===== ENUMS =====
enum LEDState
{
  OFF = LOW,
  ON = HIGH
};

enum ButtonState
{
  PRESSED = HIGH,
  UNPRESSED = LOW
};

// ===== FUNCTION PROTOTYPES =====
void handleTest();
void handleIdle();
void handleIRIdle();
void handleFollowLine();
void handleAvoidObstacle();
void initializePins();
void initializeSerialPort();
void attachServoMotors();
void zeroEncoders();
void turnLED(LEDState state);
ButtonState getButtonState();
void rotateStepperAdeg(int degrees);
void rotateStepperBdeg(int degrees);
void logError(const char *message);
void setStepperMotorSpeedsToMax();
void servosOff();
void rotateStepperAsteps(int steps);
void rotateStepperBsteps(int steps);

// ===== MAIN SETUP =====
void setup()
{
  Serial.println("Starting...");

  delay(1000);

  initializePins();
  initializeSerialPort();
  attachServoMotors();
  setStepperMotorSpeedsToMax();
  zeroEncoders();

  systemStateHandler.changeState(SystemState::IR_IDLE);
}

// ===== MAIN LOOP =====
void loop()
{
  switch (systemStateHandler.getCurrentState())
  {
  case SystemState::TEST:
    handleTest();
    break;
  case SystemState::IDLE:
    handleIdle();
    break;
  case SystemState::FOLLOW_LINE:
    handleFollowLine();
    break;
  case SystemState::AVOID_OBSTACLE:
    handleAvoidObstacle();
    break;
  case SystemState::IR_IDLE:
    handleIRIdle();
    break;
  default:
    logError("Invalid state");
    break;
  }
}

// ===== OTHER FUNCTIONS =====
void handleTest()
{
  // Code for testing

  // read the state of the encoder value:
  /*long encoderValueA = encoderA.read();
  long encoderValueB = encoderB.read();

  Serial.print("Encoder Value: ");
  Serial.print(encoderValueA);
  Serial.print(", ");
  Serial.println(encoderValueB);*/

  static unsigned long myTimerStart = millis();
  Serial.print("Time: ");
  Serial.print(millis() - myTimerStart);
  Serial.print(", ");

  if (getButtonState() == PRESSED)
  {
    turnLED(ON);
    logError("Button pressed");
  }
  else
  {
    turnLED(OFF);

    if (millis() - myTimerStart < 1000)
    {
      Serial.print("Motor A Forward");
      // Test motor A (right) FORWARD
      motorDriver.motorAForward();
      motorDriver.motorBStop();
    }
    else if (millis() - myTimerStart < 2000) 
    {
      Serial.print("Motor A Reverse");
      // Test motor A (right) REVERSE
      motorDriver.motorAReverse();
      motorDriver.motorBStop();
    }
    else if (millis() - myTimerStart < 3000)
    {
      Serial.print("Motor B Forward");
      // Test motor B (left) FORWARD
      motorDriver.motorBForward();
      motorDriver.motorAStop();
    }
    else if (millis() - myTimerStart < 4000)
    {
      Serial.print("Motor B Reverse");
      // Test motor B (left) REVERSE
      motorDriver.motorBReverse();
      motorDriver.motorAStop();
    }
    else if (millis() - myTimerStart < 9000)
    {
      Serial.print("Stepper A Forward");
      // Test stepper A (four-bar) FORWARD
      servosOff();
      rotateStepperAsteps(1);
    }
    else if (millis() - myTimerStart < 14000)
    {
      Serial.print("Stepper A Reverse");
      // Test stepper A (four-bar) REVERSE
      servosOff();
      rotateStepperAsteps(-1);
    }
    else if (millis() - myTimerStart < 16000)
    {
      Serial.print("Stepper B Forward");
      // Test stepper B (lift) FORWARD
      servosOff();
      rotateStepperBsteps(1);
    }
    else if (millis() - myTimerStart < 18000)
    {
      Serial.print("Stepper B Reverse");
      // Test stepper B (lift) REVERSE
      servosOff();
      rotateStepperBsteps(-1);
    }
    else
    {
        Serial.print("Resetting...");
        delay(5000);
        myTimerStart = millis();
    }
  }
  Serial.println();

}

void handleIdle()
{
  servosOff();
  if (getButtonState() == PRESSED)
  {
    turnLED(ON);
  }
  else
  {
    turnLED(OFF);
  }
}

void handlePIDEncoderDrive()
{
  // PID parameters
  float Kp = 1.0;
  float Ki = 0.0;
  float Kd = 0.0;

  // PID variables
  float integral = 0;
  float previousError = 0;
  // Read encoder values
  long encoderValueA = encoderA.read();
  long encoderValueB = encoderB.read();
  
  // Calculate error
  long error = encoderValueA - encoderValueB;
  
  // PID control
  integral += error;
  float derivative = error - previousError;
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  // Adjust motor speeds
  int motorSpeedA = constrain(255 - output, 0, 255);
  int motorSpeedB = constrain(255 + output, 0, 255);
  
  motorDriver.motorAForward(motorSpeedA);
  motorDriver.motorBForward(motorSpeedB);
  
  // Debugging output
  Serial.print("Left: ");
  Serial.print(encoderValueA);
  Serial.print(" Right: ");
  Serial.print(encoderValueB);
  Serial.print(" Error: ");
  Serial.println(error);
  
  // Update previous error
  previousError = error;
  
  // Short delay to avoid overwhelming the microcontroller
  delay(100);
}

void handleIRIdle()
{
  // Read the sensor values
  lineSensorA1.read();
  lineSensorA2.read();
  lineSensorA3.read();
  lineSensorB.read();

  // Calculate the averages
  int avgA1 = lineSensorA1.average();
  int avgA2 = lineSensorA2.average();
  int avgA3 = lineSensorA3.average();
  int avgB = lineSensorB.average();

  Serial.print("Line sensor: ");
  Serial.print(avgA1);
  Serial.print(", ");
  Serial.print(avgA2);
  Serial.print(", ");
  Serial.print(avgA3);
  Serial.print(", ");
  Serial.println(avgB);

  servosOff();
}

void handleFollowLine()
{
  /*if (millis() - lastStateChangeTime > stateDuration) {  // Ensure at least stateDuration has passed
    if (lineLost) {
      changeState(IDLE);
    } else if (obstacleDetected) {
      changeState(AVOID_OBSTACLE);
    }
  }*/
}

void handleAvoidObstacle()
{
  /*// Assume we need to avoid obstacles for a minimum of 3 seconds
  if (millis() - lastStateChangeTime > 3000) {  // Stay in AVOID_OBSTACLE for at least 3000 ms
    if (obstacleCleared) {
      changeState(FOLLOW_LINE);
    }
  }*/
}

// ===== HELPER FUNCTIONS =====
void initializePins()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);     // Button
  pinMode(ENCODER_PIN_A1, INPUT_PULLUP); // Encoder A
  pinMode(ENCODER_PIN_A2, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B1, INPUT_PULLUP); // Encoder B
  pinMode(ENCODER_PIN_B2, INPUT_PULLUP);
  pinMode(LINE_SENSOR_PIN_A1, INPUT); // Line Sensor A
  pinMode(LINE_SENSOR_PIN_A2, INPUT);
  pinMode(LINE_SENSOR_PIN_A3, INPUT);
  pinMode(LINE_SENSOR_PIN_B, INPUT);    // Line Sensor B
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); // Ultrasonic Sensor
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT); // LED
}

void initializeSerialPort()
{
  Serial.begin(9600); // Initialize Serial port
  while (!Serial)
    ; // Waits for the Serial port to connect.
}

void attachServoMotors()
{
  motorDriver.attachMotorA(SERVO_PIN_A1, SERVO_PIN_A2);
  motorDriver.attachMotorB(SERVO_PIN_B1, SERVO_PIN_B2);
}

void zeroEncoders()
{
  encoderA.write(0);
  encoderB.write(0);
}

void turnLED(LEDState state)
{
  digitalWrite(LED_PIN, state);
}

ButtonState getButtonState()
{
  return (!digitalRead(BUTTON_PIN)) == HIGH ? PRESSED : UNPRESSED;
}

void rotateStepperAdeg(int degrees)
{
  stepperMotorA.step(degrees / 360.0 * STEPPER_A_STEPS_PER_REVOLUTION);
}

void rotateStepperBdeg(int degrees)
{
  stepperMotorB.step(degrees / 360.0 * STEPPER_B_STEPS_PER_REVOLUTION);
}

void rotateStepperAsteps(int steps)
{
  stepperMotorA.step(steps);
}

void rotateStepperBsteps(int steps)
{
  stepperMotorB.step(steps);
}

void logError(const char *message)
{
  systemStateHandler.changeState(SystemState::IDLE);
  turnLED(ON);
  servosOff();
  Serial.println(message);
  while (true)
    ; // Stop the program
}

void setStepperMotorSpeedsToMax()
{
  stepperMotorA.setSpeed(STEPPER_A_MAX_SPEED);
  stepperMotorB.setSpeed(STEPPER_B_MAX_SPEED);
}

void servosOff()
{
  motorDriver.motorAStop();
  motorDriver.motorBStop();
}