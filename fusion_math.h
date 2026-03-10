#ifndef __FUSION_MATH_H__
#define __FUSION_MATH_H__

#include <Arduino.h>

#define Kp 20.0f
#define Ki 0.001f

typedef struct {
  float q0, q1, q2, q3;
} Quaternion;

typedef struct {
  float roll, pitch, yaw;
} EulerAngles;


void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt);
EulerAngles getEulerAngles();

#endif