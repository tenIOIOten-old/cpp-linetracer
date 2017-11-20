#ifndef My_PID
#define My_PID

//モータの最大速度（条件分岐もこれに合わせて行う）を仮に１としておく
//有効数字を３桁とする
//１００倍して少数を切れば3桁になるので１００倍している
#define MOTOR_MAX_VALUE 1*100 

// pid制御
#define KP 1
#define KI 0
#define KD 20

void pid();

#endif 