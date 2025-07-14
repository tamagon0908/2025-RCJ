#include "center.h"
#include <math.h>
#include "posture.h"
#include "NewPing.h"
#include "motor.h"
#include "ctrl.h"

// グローバル変数として定義されているセンサー値を参照するため、extern宣言は適切
extern int distance_L1;
extern int distance_L2;  // L2も姿勢補正で使う可能性を考慮
extern int distance_R;

// main.inoで定義されているNewPingオブジェクトの配列を参照
extern NewPing sonar[];

// 左右の超音波センサー間の距離（cm）
// ロボットの設計に合わせて適切な値を設定してください。
#define ROBOT_SENSOR_WIDTH 12.0  // ロボットのセンサー間の幅、正確な値に調整してください

// 次のマスまでの目標距離（cm） (この関数では直接使用しないが、参考として残しておく)
#define TARGET_DISTANCE 30.0

/**
 * @brief ロボットを次のマスの中央に誘導するための姿勢補正を行う関数
 * 壁との距離の差分から傾きを計算し、旋回と前進を行う。
 */
void center_deg_targetAngle() {
  double tolerance = 1.0;                     // ズレの許容誤差（cm）
  double angle_tolerance = 2.0;               // 角度の許容誤差（度）
  double deviation;                           // 中心からのズレ
  double target_angle_deg;                    // 目標角度
  double forward_distance = TARGET_DISTANCE;  // 次タイルまでの距離（cm）

  int max_attempts = 5;
  for (int attempt = 0; attempt < max_attempts; attempt++) {

    int current_L1 = sonar[1].ping_cm();
    int current_R = sonar[3].ping_cm();

    if (current_L1 == 0 || current_R == 0) {
      Serial.println("Warning: Invalid sensor readings in center_deg_targetAngle(). Skipping correction.");
      delay(50);
      continue;
    }

    // 中央からのズレを算出：ロボットの左右センサーが中央に等距離になるのが理想なので、
    // その差の半分が実際のズレとみなせる（L1 > R なら右寄り）
    deviation = (current_L1 - current_R) / 2.0;

    if (fabs(deviation) <= tolerance) {
      break;
    }

    // 中心へ戻るための角度を算出：θ = tan⁻¹(ずれ / 進行距離)
    target_angle_deg = degrees(atan(deviation / forward_distance));

    Serial.print("Deviation: ");
    Serial.print(deviation);
    Serial.print(" cm, Target Angle: ");
    Serial.print(target_angle_deg);
    Serial.println(" degrees");

    if (fabs(target_angle_deg) <= angle_tolerance) {
      break;
    }

    // 回転方向に応じて回転
    if (deviation > 0) {  // 右に寄ってる → 左に向けて補正
      rotateByAngle_L(fabs(target_angle_deg));
    } else {
      rotateByAngle_R(fabs(target_angle_deg));
    }

    stopMotors();
    delay(50);  // センサー更新のための安定化待ち
  }

  stopMotors();
}
