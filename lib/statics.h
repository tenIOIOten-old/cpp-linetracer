#ifndef STATICS
#define STATICS

// For　encoder
#define R 30.0f      // タイヤ半径
#define DT 0.1f      // 時間
#define PI 3.141592f // 円周率
#define PPR 400.0f   // 一周するときのカウント値

// For sensor
#define THRESHOLD 1.0f
#define VOLTAGE 3.3f

// For PID
#define REDUCE_RATIO 0.9f
#define MOTOR_MAX_VALUE 33
#define KP 0.46f
#define KI 0.00005f
#define KD 0.0f

// not use
#define B 96.0f // 車幅
#define G 38.2f // ギア比

#endif