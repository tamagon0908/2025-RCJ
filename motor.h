#ifndef motor_h
#define motor_h

#include <Arduino.h>
#include <color.h>

extern volatile int pulseCount_A;
extern volatile int pulseCount_B;
extern float rpm_A, rpm_B;

extern int distance_L1;
extern int distance_L2;

void encoderISR_A();
void encoderISR_B();
void motorSetup();
void kaiten();                         //回転数取得
void forward(int targetRevolutions);   //右だけ前進　仮引数に回転数
void just_forward();                   //ただひたすらまっすぐ
void backward(int targetRevolutions);  //左だけ前進
void forward_cm(float cm);
void backward_cm(float cm);
float kyori(int pulsecount);                         //進んだ距離
void stopMotors();                     //ストップ

#endif
