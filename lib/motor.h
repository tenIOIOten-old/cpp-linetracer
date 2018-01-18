#ifndef MOTOR
#define MOTOR

#include <mbed.h>
#include <statics.h>
class Motor
{
private:
  PwmOut verocity;
  DigitalOut direction;

public:
  Motor(PinName v, PinName d) : verocity(v), direction(d) {}
  void setPeriod(float time)
  {
    verocity.period(time);
  }
  void setVerocity(float v)
  {
    verocity = v;
  }
  float getVerocity()
  {
    return verocity.read();
  }
  // 左 true で後ろ　false で前,右はつながってない
  void setDirection(bool new_d)
  {
    direction = new_d;
  }
};

#endif