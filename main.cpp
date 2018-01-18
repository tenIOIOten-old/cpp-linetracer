// システム
#include <mbed.h>
#include <rtos.h>
#include <Eigen/Core.h>

// 自作ライブラリ
#include <sensor.h>
#include <motor.h>
#include <encoder.h>
#include <led.h>
#include <switch.h>
#include <statics.h>

using namespace Eigen;

// グローバル変数
Switch push_switch(D9);
Switch start_switch(D10);

Led running_led(D3);

Sensor sensor(A2, A1, A0);

Motor left_motor(D6, D4);
Motor right_motor(D5, D7);

Encoder left_encoder(D12);
Encoder right_encoder(D11);

Thread threadPID;
Thread threadPulse;
Thread threadMap;
Ticker tickerOdometry;

// For PID 初期化
float MOTOR_MAX_VALUE = 24.0, LEFT_RATIO = 1, RIGHT_RATIO = 1;
float KP = 0.25, KI = 0.00005, KD = 0.0;

// pid function
// ラインをトレースする
void pidCtl()
{
  // 初期化処理
  right_motor.setPeriod(0.0002f);
  left_motor.setPeriod(0.0002f);
  right_motor.setVerocity(MOTOR_MAX_VALUE / 100);
  left_motor.setVerocity(MOTOR_MAX_VALUE / 100);
  float motor_cnt_val = 0.0;
  float p_val = 0.0, i_val = 0.0, d_val = 0.0;
  float sensor_val = 0.0, prev_sensor_val = 0;

  Thread::wait(2000);
  while (1)
  {
    // detect sensor value
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
      sensor_val = i_val = 0.0f;
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
    // ０以下なら左に曲がるため左の速度を減らす、引数値分最大速度から減る.０にはならないようにする
    if (motor_cnt_val < 0)
    {
      if (MOTOR_MAX_VALUE + motor_cnt_val < 0.0)
      {
        left_motor.setVerocity(MOTOR_MAX_VALUE / 100 / 1.5);
        i_val -= p_val;
      }
      else
      {
        left_motor.setVerocity((MOTOR_MAX_VALUE + motor_cnt_val) / 100 / LEFT_RATIO);
      }
      right_motor.setVerocity(MOTOR_MAX_VALUE / 100 / RIGHT_RATIO);
    }
    // ０以上なら右に曲がるため右の速度を減らす
    else if (motor_cnt_val > 0)
    {
      if (MOTOR_MAX_VALUE - motor_cnt_val < 0.0)
      {
        right_motor.setVerocity(MOTOR_MAX_VALUE / 100 / 1.5);
        i_val -= p_val;
      }
      else
      {
        right_motor.setVerocity((MOTOR_MAX_VALUE - motor_cnt_val) / 100 / RIGHT_RATIO);
      }
      left_motor.setVerocity(MOTOR_MAX_VALUE / 100 / LEFT_RATIO);
    }

    if (push_switch.read())
    {
      left_motor.setDirection(true);
      left_motor.setVerocity(0.4);
      right_motor.setVerocity(0.4);
      Thread::wait(750);
      left_motor.setDirection(false);
      left_motor.setVerocity(0.4);
    }
  }
}
// pulse function
// 周波数を数える
void pulseCount()
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
// odometry function
// 位置情報を計算
// 1*3の行列作成
Vector3f vw(0, 0, 0); // タイヤの回転速度
Vector3f vk(0, 0, 0); // Vehicle velocity (v, omega, 1)
Vector3f p(0, 0, 0);  // Position in world coordinates (x, y, yaw)

// 3*3の行列作成、速度(vk)計算用
Matrix3f vM;

// 3*3の行列作成、座標計算用
Matrix3f M;

void odometryCtl()
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
// map function
// 位置情報から制御
void mapCtl()
{
  while (1)
  {
    if (
        p[0] >= 500.0f && p[1] >= 100.0 && p[1] < 200.0f)
    {
      LEFT_RATIO = 1;
      RIGHT_RATIO = 1.5;
    }
    else if (
        p[0] >= 500.0f && p[0] < 100 && p[1] >= 0.0 && p[1] < 300.0f)
    {
    }
    else
    {
      LEFT_RATIO = 1;
      RIGHT_RATIO = 1;
    }
  }
}
// メイン関数
int main()
{

  // スイッチがオンになるまで消灯。
  running_led.off();
  while (start_switch.read() == 0)
    ;
  // スレッド管理
  threadPID.start(pidCtl);
  threadPulse.start(pulseCount);
  // threadMap.start(mapCtl);
  tickerOdometry.attach(odometryCtl, DT);
  // 点灯
  running_led.on();

  // メインスレッド
  while (1)
  {
    // スイッチオンのときループを抜けて、点灯。
    while (start_switch.read() == 0)
    {
      running_led.off();
      right_motor.setVerocity(0.0);
      left_motor.setVerocity(0.0);
    }
    running_led.on();
    // wait(1.0);
    // printf("left: %f\t right: %f\t, x: %f\t, y: %f\n\r", left_motor.getVerocity(), right_motor.getVerocity(), p[0], p[1]);
  }
}