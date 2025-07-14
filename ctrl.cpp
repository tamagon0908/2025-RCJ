#include "gyro.h"
#include "motor.h"

#define IN1_A 32
#define IN2_A 34
#define IN1_B 33
#define IN2_B 35
#define IN1_C 36
#define IN2_C 38
#define IN1_D 37
#define IN2_D 39

extern int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void rotateByAngle_R(float targetAngle) {
  float angle = 0.0;
  unsigned long prevTime = millis();

  // 右旋回（モータの回転方向を個別に指定）
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, HIGH);
  digitalWrite(IN1_B, HIGH);
  digitalWrite(IN2_B, LOW);
  digitalWrite(IN1_C, LOW);
  digitalWrite(IN2_C, HIGH);
  digitalWrite(IN1_D, HIGH);
  digitalWrite(IN2_D, LOW);

  while (abs(angle) < abs(targetAngle)) {
    gyro();  // ジャイロデータを更新

    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    float angularVelocity = GyZ / 131.0;  // MPU6050ならこのスケーリング
    angle += angularVelocity * dt;
  }

  stopMotors();
}

  void rotateByAngle_L(float targetAngle) {
  float angle = 0.0;
  unsigned long prevTime = millis();

  // 右旋回（モータの回転方向を個別に指定）
  digitalWrite(IN1_A, HIGH);
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN1_B, LOW);
  digitalWrite(IN2_B, HIGH);
  digitalWrite(IN1_C, HIGH);
  digitalWrite(IN2_C, LOW);
  digitalWrite(IN1_D, LOW);
  digitalWrite(IN2_D, HIGH);

  while (abs(angle) < abs(targetAngle)) {
    gyro();  // ジャイロデータを更新

    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    float angularVelocity = GyZ / 131.0;  // MPU6050ならこのスケーリング
    angle += angularVelocity * dt;
  }

  // モータ停止
  stopMotors();
}
