#ifndef CUSTOMSTEPPER_H
#define CUSTOMSTEPPER_H

#include <Arduino.h>

class CustomStepper {
    public:
        // Constructor to initialize the stepper motor pins and steps per revolution
        CustomStepper(int dirPin, int stepPin, int enablePin, int stepsPerRev);
        
        // Method to rotate the motor clockwise
        void rotateClockwise(int numSteps, int rpm);
        
        // Method to rotate the motor counterclockwise
        void rotateCounterclockwise(int numSteps, int rpm);

    private:
        int dirPin;
        int stepPin;
        int enablePin;
        int stepsPerRev;

        // Method to calculate the delay in microseconds based on RPM
        int calculateDelayMicroseconds(int rpm);
        
        // Method to step the motor
        void stepMotor(int delayMicroseconds);
};

#endif // CUSTOMSTEPPER_H
