#ifndef SERIALCONTROLLER_H
#define SERIALCONTROLLER_H

#include <Arduino.h>

class SerialController {
public:
    // Constructor
    SerialController();

    // Destructor
    ~SerialController();

    // Other member functions
    void init();
    void printBinaryWithLeadingZeros(byte number);
    void printWithTimestamp(const char *message);
    void printlnWithTimestamp(const char *message);
private:
    // Private member variables
    unsigned long lastPrintTime = 0;

};

#endif // SERIALCONTROLLER_H