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
  float getVerociry()
  {
    return verocity.read();
  }

  // 引数分だけ設定最大速度から減らす
  void reduceVerocity(float motor_cnt_val = 0.0f)
  {
    verocity = (MOTOR_MAX_VALUE + motor_cnt_val) / 100.0f;
  }
  // true で後ろ　false で前
  void setDirection(bool new_d)
  {
    direction = new_d;
  }
};

#endif