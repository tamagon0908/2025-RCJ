#include "motor.h"
#include "Arduino.h"
#include "NewPing.h"
#include "color.h"
#include "Adafruit_TCS34725.h"
#include "posture.h"

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

volatile int pulseCount_A = 0;  // RPM用（1秒ごとにリセット）
volatile int pulseCount_B = 0;
volatile long totalPulse_A = 0;  // 累積パルス数（forwardなどに使用）
volatile long totalPulse_B = 0;

float rpm_A = 0.0, rpm_B = 0.0;
const int pulsesPerRevolution = 12;
const float gearRatio = 75.0;
unsigned long prevTime = 0;

const float wheel = 6.0;
const float dis_per = wheel * 3.1415;

extern int distance_L1;
extern int distance_L2;
extern int distance_R;

extern Adafruit_TCS34725 tcs;

extern NewPing sonar[];

void encoderISR_A() {
  pulseCount_A++;
  totalPulse_A++;
}

void encoderISR_B() {
  pulseCount_B++;
  totalPulse_B++;
}

void motorSetup() {
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);
  pinMode(IN1_C, OUTPUT);
  pinMode(IN2_C, OUTPUT);
  pinMode(IN1_D, OUTPUT);
  pinMode(IN2_D, OUTPUT);

  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B1, INPUT);
  pinMode(ENCODER_B2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoderISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoderISR_B, RISING);

  Serial.begin(9600);
}

void kaiten() {
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= 1000) {
    float raw_rpm_A = (pulseCount_A > 0) ? (pulseCount_A / (float)pulsesPerRevolution) * 60.0 : 0;
    float raw_rpm_B = (pulseCount_B > 0) ? (pulseCount_B / (float)pulsesPerRevolution) * 60.0 : 0;

    rpm_A = raw_rpm_A / gearRatio;
    rpm_B = raw_rpm_B / gearRatio;

    Serial.print("RPM_右: ");
    Serial.println(rpm_A > 0 ? rpm_A : 0.0);

    Serial.print("RPM_左: ");
    Serial.println(rpm_B > 0 ? rpm_B : 0.0);

    pulseCount_A = 0;
    pulseCount_B = 0;
    prevTime = currentTime;
  }
}

void forward(int targetRevolutions) {
  long targetPulses = (long)targetRevolutions * pulsesPerRevolution;
  totalPulse_A = 0;
  totalPulse_B = 0;

  long startPulse_A = totalPulse_A;
  long startPulse_B = totalPulse_B;

  digitalWrite(IN1_A, HIGH);
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN1_B, HIGH);
  digitalWrite(IN2_B, LOW);
  digitalWrite(IN1_C, HIGH);
  digitalWrite(IN2_C, LOW);
  digitalWrite(IN1_D, HIGH);
  digitalWrite(IN2_D, LOW);

  while (true) {
    kaiten();
    unsigned int distance_F = sonar[0].ping_cm();
    if (distance_F < 10) {
      break;
    }

    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    bool black = (r < 500 && r > 450 && g > 350 && g < 250 && b > 250 && b < 350);  // この条件は要調整

    if (black) {
      long traveledPulses = ((totalPulse_A - startPulse_A) + (totalPulse_B - startPulse_B)) / 2;
      stopMotors();

      backward(traveledPulses / pulsesPerRevolution);
      posture_wall();
      break;
    }

    if (totalPulse_A >= targetPulses || totalPulse_B >= targetPulses) {
      break;
    }
  }

  stopMotors();
}


//使えたら使いたい　ギヤ比わかれば行けるかも
// void forward_cm(float cm) {    
//   float revolutions = (cm / dis_per) * gearRatio;
//   forward(revolutions);
//   stopMotors();
// }

// void backward_cm(float cm) {
//   float revolutions = (cm / dis_per) * gearRatio;
//   backward(revolutions);
//   stopMotors();
// }

void backward(int targetRevolutions) {
  long targetPulses = (long)targetRevolutions * pulsesPerRevolution;
  totalPulse_A = 0;
  totalPulse_B = 0;

  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, HIGH);
  digitalWrite(IN1_B, LOW);
  digitalWrite(IN2_B, HIGH);
  digitalWrite(IN1_C, LOW);
  digitalWrite(IN2_C, HIGH);
  digitalWrite(IN1_D, LOW);
  digitalWrite(IN2_D, HIGH);

  while (true) {
    kaiten();
    if (totalPulse_A >= targetPulses || totalPulse_B >= targetPulses) {
      break;
    }
  }
  stopMotors();
}

void stopMotors() {
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN1_B, LOW);
  digitalWrite(IN2_B, LOW);
  digitalWrite(IN1_C, LOW);
  digitalWrite(IN2_C, LOW);
  digitalWrite(IN1_D, LOW);
  digitalWrite(IN2_D, LOW);
}
