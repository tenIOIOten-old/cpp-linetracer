#ifndef LED
#define LED

#include <mbed.h>

class Led
{
private:
  DigitalOut output;

public:
  Led(PinName pin) : output(pin) {}
  void on()
  {
    output = 1;
  }
  void off()
  {
    output = 0;
  }
  int getValue()
  {
    return output.read();
  }
};

#endif