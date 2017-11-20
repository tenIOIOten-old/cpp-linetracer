//
//  main.cpp ...　位置座標を計算する
//
//  2017.10.26 ... Shiratori Tenta
//
#include <mbed.h>
#include <odmetory.h>
#include <linetrace_pid.h>

//　入力インスタンスと時間インスタンス作成
encoda pulseL(); // ＊書き直しといて
encoda pulseR(); // ＊書き直しといて
Switch startSwich(); // ＊書き直しといて
Ticker ticker;

DigitalOut myled(LED1);

int main() {
    ticker(tickDistance(),DT);
    while(){
    }
    while(1) {
        myled = 1; // LED is ON
        wait(0.2); // 200 ms
        myled = 0; // LED is OFF
        wait(1.0); // 1 sec
    }
}
