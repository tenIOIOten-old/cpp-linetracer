#include <odmetory.h>
#include <Eigen/Core.h> // 行列計算用のライブラリ
using namespace std;
using namespace Eigen;

// 3*3の行列作成、速度(vk)計算用
Matrix3f vM;
// 3*3の行列作成、座標計算用
Matrix3f M;

// 1*3の行列作成
Vector3f vw(0, 0, 0); // タイヤの回転速度
Vector3f vk(0, 0, 0); // Vehicle velocity (v, omega, 1)
Vector3f p(0, 0, 0);  // Position in world coordinates (x, y, yaw)


// タイヤの速度計算
double culcVelocity(float p)
{
  return 2 * PI * r * G * (p / PPR * dt);
}
// 座標計算

int tickDistance()
{
    // カウンタインクリメント
  // カウンタ
  static int i = 0;
  
  vM << 0.5, 0.5, 0, // 速度計算用マトリックス初期化
  1 / B, 1 / B, 0,
  0.0, 0.0, 1;

  M << dt * cos(p[2]), 0, p[0], // 座標計算用マトリックス初期化
  dt * sin(p[2]), 0, p[1],
  0, dt, p[2];
  
  // タイヤの速度計算
  vl = culcVelocity(pulseL);
  vr = culcVelocity(pulseR);

  vw << vl, vr, 1;
  vk = vM * vw;
  p = M * vk; // 座標確定

  fprintf(stdout, "%f, %f, %f, %f\n", i * dt, p[0], p[1], p[2]); // 出力
  i++; 
}