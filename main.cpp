#include <mbed.h>
#include <rtos.h>
#include <Eigen/Core.h>

#include <sensor.h>
#include <motor.h>
#include <encoder.h>
#include <led.h>
#include <switch.h>
#include <statics.h>

using namespace Eigen;

Switch start_switch(D1);
Switch push_switch(D2);
Led running_led(D3);

Sensor sensor(A2, A1, A0);

Motor left_motor(D5, D4);
Motor right_motor(D6, D7);

Encoder left_encoder(D12);
Encoder right_encoder(D11);

Thread threadPID;
Thread threadPulse;
Ticker tickerOdometry;

// pid
void pid();
float motor_cnt_val = 0.0;

// pulse
void pulse();

// odometry
void odometry();
// 1*3の行列作成
Vector3f vw(0, 0, 0); // タイヤの回転速度
Vector3f vk(0, 0, 0); // Vehicle velocity (v, omega, 1)
Vector3f p(0, 0, 0);  // Position in world coordinates (x, y, yaw)

// 3*3の行列作成、速度(vk)計算用
Matrix3f vM;

// 3*3の行列作成、座標計算用

Matrix3f M;

int main()
{
  // 初期化処理
  right_motor.setPeriod(0.0002f);
  left_motor.setPeriod(0.0002f);
  right_motor.setVerocity(0.0f);
  left_motor.setVerocity(0.0f);

  running_led.off();

  // スイッチオンのときループを抜けて、点灯。
  while (start_switch.read())
    ;

  threadPID.start(pid);
  threadPulse.start(pulse);
  tickerOdometry.attach(odometry, DT);

  running_led.on();

  while (1)
  {
  }
}
void pid()
{
  float p_val = 0.0, i_val = 0.0, d_val = 0.0;
  float sensor_val = 0.0, prev_sensor_val = 0;
  while (1)
  {
    switch (sensor.checkValue())
    {
    case 0b001: // w w b
      sensor_val = MOTOR_MAX_VALUE * 1.0f;
      break;
    case 0b011: // w b b
      sensor_val = MOTOR_MAX_VALUE * 0.5f;
      break;
    case 0b111: // b b b
    case 0b010: // w b w
      prev_sensor_val = sensor_val = i_val = 0.0;
      break;
    case 0b110: // b b w
      sensor_val = MOTOR_MAX_VALUE * -0.5f;
      break;
    case 0b100: // b w w
      sensor_val = MOTOR_MAX_VALUE * -1.0f;
      break;
    default: // w w w
      sensor_val = prev_sensor_val;
      break;
    }
    // pid culculation
    prev_sensor_val = sensor_val;
    p_val = sensor_val;
    i_val += p_val;
    d_val = sensor_val - prev_sensor_val;

    motor_cnt_val = (p_val * KP) + (i_val * KI) + (d_val * KD);
  }
}
void pulse()
{
  while (1)
  {
    if (left_encoder.isChange())
    {
      left_encoder.incrementPulse();
    }
    if (right_encoder.isChange())
    {
      right_encoder.incrementPulse();
    }
    wait(0.00001);
  }
}
void odometry()
{
  // 行列定義
  vM << 0.5, 0.5, 0,
      1 / B, -1 / B, 0,
      0.0, 0.0, 1;

  M << DT * cos(p[2]), 0, p[0],
      DT * sin(p[2]), 0, p[1],
      0, DT, p[2];

  vw << left_encoder.getVerocity(), right_encoder.getVerocity(), 1;
  left_encoder.resetPulse();
  right_encoder.resetPulse();

  vk = vM * vw;
  p = M * vk; // 座標確定
}