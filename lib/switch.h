#ifndef SWITCH
#define SWITCH

#include <mbed.h>

class Switch
{
private:
  DigitalIn input;

public:
  Switch(PinName pin) : input(pin) {}
  int read()
  {
    return input.read();
  }
};

#endif