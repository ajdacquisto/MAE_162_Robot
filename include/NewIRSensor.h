#ifndef NEWIRSENSOR_H
#define NEWIRSENSOR_H

#include <Arduino.h>

/**
 * @brief The NewIRSensor class represents an infrared sensor with multiple pins.
 */
class NewIRSensor {
public:
    /**
     * @brief Constructs a NewIRSensor object with the specified pin numbers.
     * 
     * @param pin1 The pin number for the first sensor.
     * @param pin2 The pin number for the second sensor.
     * @param pin3 The pin number for the third sensor.
     * @param pin4 The pin number for the fourth sensor.
     * @param pin5 The pin number for the fifth sensor.
     */
    NewIRSensor(int pin1, int pin2, int pin3, int pin4, int pin5);

    /**
     * @brief Destroys the NewIRSensor object.
     */
    ~NewIRSensor();

    /**
     * @brief Initializes the infrared sensor.
     */
    void init();

    /**
     * @brief Reads the sensor data and returns the result.
     * 
     * @return The sensor data as an 8-bit unsigned integer.
     */
    uint8_t readSensorData();

private:
    // Member variables
    int pin1_; /**< The pin number for the first sensor. */
    int pin2_; /**< The pin number for the second sensor. */
    int pin3_; /**< The pin number for the third sensor. */
    int pin4_; /**< The pin number for the fourth sensor. */
    int pin5_; /**< The pin number for the fifth sensor. */
};

#endif // NEWIRSENSOR_H