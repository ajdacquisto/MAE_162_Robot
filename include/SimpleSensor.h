#ifndef SIMPLESENSOR_H
#define SIMPLESENSOR_H

class SimpleSensor {
private:
  int pin;
  int lastReading = 0;

public:
  SimpleSensor(int pin);
  void read();
  int cleanRead();
  int getLastReading();
};

#endif // SIMPLESENSOR_H