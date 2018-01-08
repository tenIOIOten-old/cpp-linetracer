#ifndef SENSOR
#define SENSOR

#include <mbed.h>
#include <statics.h>
class Sensor
{
private:
  AnalogIn left;
  AnalogIn center;
  AnalogIn right;

public:
  Sensor(PinName l, PinName c, PinName r)
      : left(l), center(c), right(r) {}
  // 100 010 001 の感じで値を返す.黒のときに１
  int checkValue()
  {
    int result = 0b000;
    if (left.read() * VOLTAGE < THRESHOLD)
      result += 0b100;
    if (center.read() * VOLTAGE < THRESHOLD)
      result += 0b010;
    if (right.read() * VOLTAGE < THRESHOLD)
      result += 0b001;
    return result;
  }
};

#endif