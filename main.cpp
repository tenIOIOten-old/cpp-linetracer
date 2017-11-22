//
//  目標
//  スレッドを立てる
//  起動から走行開始、スレッド立てまで、main
//  thread
//  →センサー
//  →モーター二種
//  →エンコーダ,オドメトリ
//
//  main.cpp ...　位置座標を計算する
//
//  2017.10.26 ... Shiratori Tenta
//
#include <mbed.h>
#include <rtos.h>

DigitalOut powerLed(LED1);
DigitalOut runningLed(LED1);

int main()
{
    Thread threadMotorL();
    Thread threadMotorR();
    Thread threadEncoder();
    Thread threadOdmetory();

    RtoTimer timerSensor();
    timerSensor.start(5);

    powerLed = 1;
    while (startSwich.read() == 0)
    {
    }
    runningLed = 1;

    wait(1);

    while (1)
    {
        threadMotorL.signal_set(0x01);
        threadMotorR.signal_set(0x01);
        threadEncoder.signal_set(0x01);
        threadOdmetory.signal_set(0x01);
    }
}
