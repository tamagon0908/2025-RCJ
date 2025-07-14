//forward(109)で30cm直進
//rotateByAngle_L(75)で左90ど

#include "motor.h"  //モータ
#include "gyro.h"   //ジャイロ
#include "ctrl.h"   //指定角度回転
#include "posture.h"
#include "Arduino.h"
#include "NewPing.h"
#include <Wire.h>
#include "center.h"
#include "Adafruit_TCS34725.h"

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

#define IN1_A 32
#define IN2_A 34
#define IN1_B 33
#define IN2_B 35
#define IN1_C 36
#define IN2_C 38
#define IN1_D 37
#define IN2_D 39

#define ENCODER_A1 2
#define ENCODER_A2 3
#define ENCODER_B1 18
#define ENCODER_B2 19

#define NUM_SENSORS 4
#define MAX_DISTANCE 200

// 超音波センサのピン定義
const uint8_t TRIGGER_PINS[NUM_SENSORS] = { 10, 12, 6, 8 };  // F, L1, L2, R
const uint8_t ECHO_PINS[NUM_SENSORS] = { 11, 13, 7, 9 };

// センサオブジェクトの配列
NewPing sonar[NUM_SENSORS] = {
  NewPing(TRIGGER_PINS[0], ECHO_PINS[0], MAX_DISTANCE),
  NewPing(TRIGGER_PINS[1], ECHO_PINS[1], MAX_DISTANCE),
  NewPing(TRIGGER_PINS[2], ECHO_PINS[2], MAX_DISTANCE),
  NewPing(TRIGGER_PINS[3], ECHO_PINS[3], MAX_DISTANCE)
};

int sikii = 20;  // 壁とみなす距離(cm)

bool wall_F = false;
bool wall_L = false;
bool wall_R = false;

bool balck  = false;
bool silver = false;
bool blue   = false;

int distance_L1;
int distance_L2;
int distance_R;
int distance_F;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

void setup() {
  gyroSetup();
  motorSetup();
  tcs.begin();
  // colorSetup();
  Serial.begin(9600);
}

void loop() {
  distance_L1 = 0.0;
  distance_L2 = 0.0;
  distance_R = 0.0;
  distance_F = 0.0;

  unsigned int distance[NUM_SENSORS];

  for (int i = 0; i < NUM_SENSORS; i++) {
    delay(50);
    distance[i] = sonar[i].ping_cm();
  }

  distance_F = distance[0];
  distance_L1 = distance[1];
  distance_L2 = distance[2];
  distance_R = distance[3];

  // 左右の壁を検知するためにL1とL2の両方を使用
  // L1 (TRIGGER_PINS[1] = 12, ECHO_PINS[1] = 13) と L2 (TRIGGER_PINS[2] = 6, ECHO_PINS[2] = 7)
  // のどちらか、または両方が壁を検知した場合に左に壁があるとみなすように調整する
  wall_F = (distance[0] > 0 && distance[0] < sikii);
  wall_L = ((distance[1] > 0 && distance[1] < sikii) || (distance[2] > 0 && distance[2] < sikii));
  wall_R = (distance[3] > 0 && distance[3] < sikii);

  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  bool black=(r<500 && r>450 && g>350 && g<250 && b>250 && b<350); //条件式を記述
  //silver = (~~~~);
  //blue =(~~~~);

  Serial.print("F: ");
  Serial.print(distance[0]);
  Serial.print(" L1: ");
  Serial.print(distance[1]);
  Serial.print(" L2: ");
  Serial.print(distance[2]);
  Serial.print(" R: ");
  Serial.println(distance[3]);

  // --- 左手法の移動ロジック ---
  // 優先順位：左 > 前 > 右 > 後ろ
  if (!wall_L) {
    // 1. 左に壁がない場合
    stopMotors();
    rotateByAngle_L(65);  // 90度左に回転
    stopMotors();
    delay(100);    // 一旦停止
    forward(109);  // 前に進む (1マス分)
    stopMotors();
    posture_wall();  // 壁に寄せて姿勢を補正
    // stopMotors();
    center_deg();  // 向きを補正
    stopMotors();
  } else if (!wall_F) {
    // 2. 左に壁があり、前方に壁がない場合
    stopMotors();
    forward(109);  // 前に進む (1マス分)
    stopMotors();
    posture_wall();
    // stopMotors();
    center_deg();
    stopMotors();
  } else if (!wall_R) {
    // 3. 左と前方に壁があり、右に壁がない場合
    stopMotors();
    rotateByAngle_R(65);  // 90度右に回転
    forward(109);         // 前に進む (1マス分)
    stopMotors();
    posture_wall();
    // stopMotors();
    center_deg();
    stopMotors();
  } else {
    // 4. 左、前、右に壁があり、行き止まりの場合
    stopMotors();
    rotateByAngle_R(130);  // 180度回転
    // 前進はしない。次のループで向きが変わった状態で再度判断する
    stopMotors();
    posture_wall();
    // stopMotors();
    center_deg();
    stopMotors();
  }
}
