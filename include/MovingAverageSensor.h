#ifndef MOVINGAVERAGESENSOR_H
#define MOVINGAVERAGESENSOR_H

#define BUFFER_SIZE 1 // Define the buffer size here

class MovingAverageSensor {
private:
  int values[BUFFER_SIZE] = {0};
  int index = 0;
  int pin;
  int lastReading = 0;

public:
  MovingAverageSensor(int pin);
  void read();
  int average() const;
  int cleanRead();
  int getLastReading();
};

#endif // MOVINGAVERAGESENSOR_H