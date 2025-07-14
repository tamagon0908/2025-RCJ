#include <math.h>
#include "posture.h"
#include "NewPing.h"
#include "motor.h"
#include "ctrl.h"

#define IN1_A 32
#define IN2_A 34
#define IN1_B 33
#define IN2_B 35
#define IN1_C 36
#define IN2_C 38
#define IN1_D 37
#define IN2_D 39

extern int distance_L1;
extern int distance_L2;
extern int distance_R;

extern NewPing sonar[];

void posture_wall() {
  double differ;
  const double ALIGNMENT_THRESHOLD = 0.5;  // 整列の許容誤差 (cm)

  // ループ内でセンサーの値を繰り返し読み取り、差が許容誤差内になるまで調整
  do {
    // ここで最新のセンサー値を読み取る
    delay(50);  // センサーの干渉を防ぐための短い遅延
    distance_L1 = sonar[1].ping_cm();
    distance_L2 = sonar[2].ping_cm();

    // センサーが0を返す（範囲外など）場合の処理
    if (distance_L1 == 0) distance_L1 = 999;  // 非常に大きい値として扱う
    if (distance_L2 == 0) distance_L2 = 999;

    differ = distance_L1 - distance_L2;  // 新しい差を計算

    if (distance_L1 > 40 || distance_L2 > 40) {
      break;
    }
    
    else if (fabs(differ) > ALIGNMENT_THRESHOLD && distance_R < 30) {  // 差が許容誤差より大きいかつ右に壁がある場合
      if (distance_L1 > distance_L2) {
        // 左のセンサーが遠い場合、わずかに右に回転
        digitalWrite(IN1_A, HIGH);
        digitalWrite(IN2_A, LOW);
        digitalWrite(IN1_B, LOW);
        digitalWrite(IN2_B, HIGH);
        digitalWrite(IN1_C, HIGH);
        digitalWrite(IN2_C, LOW);
        digitalWrite(IN1_D, LOW);
        digitalWrite(IN2_D, HIGH);
      } else {  // distance_L1 < distance_L2
        // 右のセンサーが遠い場合、わずかに左に回転
        digitalWrite(IN1_A, LOW);
        digitalWrite(IN2_A, HIGH);
        digitalWrite(IN1_B, HIGH);
        digitalWrite(IN2_B, LOW);
        digitalWrite(IN1_C, LOW);
        digitalWrite(IN2_C, HIGH);
        digitalWrite(IN1_D, HIGH);
        digitalWrite(IN2_D, LOW);
      }
      delay(100);    // わずかに回転させる時間
      stopMotors();  // モーターを一旦停止し、次の測定と調整に備える
    } else {
      // 差が許容誤差内になったらループを抜ける
      break;
    }
    delay(50);     // センサーが安定し、ロボットが少し動くための時間
  } while (true);  // 条件が満たされるまで無限ループ

  stopMotors();  // 最終的にモーターを停止
}
