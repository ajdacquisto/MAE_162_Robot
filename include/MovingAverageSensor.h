#ifndef MOVINGAVERAGESENSOR_H
#define MOVINGAVERAGESENSOR_H

#define BUFFER_SIZE 15  // Define the buffer size here

class MovingAverageSensor {
private:
  int values[BUFFER_SIZE] = {0};
  int index = 0;
  int pin;

public:
  MovingAverageSensor(int pin);
  void read();
  int average() const;
};

#endif // MOVINGAVERAGESENSOR_H