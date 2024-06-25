#include <MPU6050_tockn.h>
#include <Wire.h>
#include "Motor.h"
#include "Range.h"
#define SW1 2
#define LED1 3

Range s_left(S1, -64.628, 88919.4, -68.007);
Range s_center(S2, -15.1576, 59462.6, -48.5996);
Range s_right(S3, -76.8597, 76759.9, 17.5771);

MPU6050 gyro(Wire);
Motor m1(MT1_IF, MT1_IB);
Motor m2(MT2_IF, MT2_IB);
float omega_Z;

void setup() {
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);
  Wire.begin();
  gyro.begin();
  gyro.calcGyroOffsets(true);
  m1.brake();
  m2.brake();
  pinMode(SW1, INPUT);
  while (digitalRead(SW1) == LOW) {
    gyro.update();
    omega_Z = gyro.getGyroZ();
    if (-0.5 < omega_Z && omega_Z < 0.5) {
      digitalWrite(LED1, HIGH);
    } else {
      digitalWrite(LED1, LOW);
    }
  }
}
void loop() {
  int /*speed_L,*/ speed_R;
  float speed_L;
  int speed_fw = 130;
  int steer, steer1;
  float K = 1.53;
  float omega_ref, omega_ref1;
  float range_L = s_left.read();
  float range_R = s_right.read();
  float range_C = s_center.read();
  float range_limit = 150.0;

  gyro.update();
  omega_Z = gyro.getGyroZ();
  if (range_L < 175) {
    omega_ref = -5.0 * (range_limit - range_L);
  } else if (range_R < range_limit) {
    omega_ref = 5.0 * (range_limit - range_R);
  } else {
    omega_ref = 0.0;
    omega_ref1 = 0.0;
  }
  steer = (int)(omega_ref - omega_Z) * K;
  speed_L = speed_fw - steer;
  speed_R = speed_fw + steer;
  if (range_C < 250.0) {
    omega_ref1 = -5.0 * (range_limit - range_C);
    steer1 = (int)(omega_ref1 - omega_Z);
    speed_L = 0.0;
    speed_R = 0;
    speed_L = speed_fw + steer1 + 21.0;
    speed_R = speed_fw - steer1;
  }
  m1.run(speed_L);
  m2.run(-speed_R);
}