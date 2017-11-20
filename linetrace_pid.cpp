
#include "mbed.h"
#include <linetrace_pid.h>

// 入出力
AnalogIn L_SENSOR(A0);
AnalogIn R_SENSOR(A1);
AnalogIn C_SENSOR(A2);
PwmOut L_MOTOR_V(D12);
PwmOut R_MOTOR_V(D2);

// 設定値
int motor_cnt_val = 0;

// pid計算
void pid()
{
  float l_sensor = L_SENSOR.read();
  float r_sensor = R_SENSOR.read();
  float c_sensor = C_SENSOR.read();

  float sensor_val = 0.0;
  float p_val, d_val, pid_val;
  static float i_val, prev_sensor_val = 0;

  // 条件により６段階評価、全てのセンサが白なら前回の値から評価
  if (l_sensor >= &&l_sensor <= &&r_sensor >= &&r_sensor <= &&c_sensor >= &&c_sensor <=)
  {
    sensor_val = 100.0;
  }
  else if (l_sensor >= &&l_sensor <= &&r_sensor >= &&r_sensor <= &&c_sensor >= &&c_sensor <=)
  {
    sensor_val = 50.0;
  }
  else if (l_sensor >= &&l_sensor <= &&r_sensor >= &&r_sensor <= &&c_sensor >= &&c_sensor <=)
  {
    sensor_val = 0.0;
  }
  else if (l_sensor >= &&l_sensor <= &&r_sensor >= &&r_sensor <= &&c_sensor >= &&c_sensor <=)
  {
    sensor_val = -50.0;
  }
  else if (l_sensor >= &&l_sensor <= &&r_sensor >= &&r_sensor <= &&c_sensor >= &&c_sensor <=)
  {
    sensor_val = -100.0;
  }
  else
  {
    sensor_val = prev_sensor_val;
  }

  p_val = sensor_val;
  i_val += p_val;
  d_val = sensor_val - prev_sensor_val;

  pid_val = (p_val * (KP)) + (i_val * (KI)) + (d_val * (KD));

  motor_cnt_val = (int)(pid_val);
}
