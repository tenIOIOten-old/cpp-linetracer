#ifndef ENCODER
#define ENCODER

#include <mbed.h>
#include <statics.h>
class Encoder
{
private:
  DigitalIn input;
  int before;
  int after;
  float pulse;

public:
  Encoder(PinName pin) : input(pin)
  {
    after = 0;
    before = 0;
    pulse = 0.0;
  }
  int isChange()
  {
    after = getValue();
    if (after != before)
    {
      before = after;
      return 1;
    }
    else
    {
      return 0;
    }
  }
  int getValue()
  {
    return input.read();
  }
  float getPulse()
  {
    return pulse;
  }
  void incrementPulse()
  {
    pulse += 1.0f;
  }
  void resetPulse()
  {
    pulse = 0;
  }
  float getVerocity()
  {
    return 2 * PI * R * pulse / DT / PPR;
  }
};

#endif