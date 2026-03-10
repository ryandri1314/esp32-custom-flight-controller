#include "fusion_math.h" 

static Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; 
  ax /= norm; ay /= norm; az /= norm;

  vx = 2 * (q.q1*q.q3 - q.q0*q.q2);
  vy = 2 * (q.q0*q.q1 + q.q2*q.q3);
  vz = q.q0*q.q0 - q.q1*q.q1 - q.q2*q.q2 + q.q3*q.q3;

  if (norm > 0.6 && norm < 1.4) {
    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;
  } else {
    ex = 0.0f;
    ey = 0.0f;
    ez = 0.0f;
  }

  if (Ki > 0) {
    integralFBx += Ki*ex*dt;
    integralFBy += Ki*ey*dt;
    integralFBz += Ki*ez*dt;
  }

  gx += (Kp*ex + integralFBx);
  gy += (Kp*ey + integralFBy);
  gz += (Kp*ez + integralFBz);

  q.q0 += (-q.q1*gx - q.q2*gy - q.q3*gz) * (0.5f * dt);
  q.q1 += (q.q0*gx + q.q2*gz - q.q3*gy) * (0.5f * dt);
  q.q2 += (q.q0*gy - q.q1*gz + q.q3*gx) * (0.5f * dt);
  q.q3 += (q.q0*gz + q.q1*gy - q.q2*gx) * (0.5f * dt);

  norm = sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);

  if (norm < 1e-6f) {
    q.q0 = 1.0f;
    q.q1 = 0.0f;
    q.q2 = 0.0f;
    q.q3 = 0.0f;
    return;
  }
  q.q0 /= norm;
  q.q1 /= norm;
  q.q2 /= norm;
  q.q3 /= norm; 
}

EulerAngles getEulerAngles() {
  EulerAngles angles;

  // Roll (x-axis)
  float sinr_cosp = 2 * (q.q0 * q.q1 + q.q2 * q.q3);
  float cosr_cosp = 1 - 2 * (q.q1 * q.q1 + q.q2 * q.q2);
  angles.roll = atan2(sinr_cosp, cosr_cosp) * 57.29578f;

  // Pitch (y-axis)
  float sinp = 2 * (q.q0 * q.q2 - q.q3 * q.q1);
  if (abs(sinp) >= 1)
    angles.pitch = copysign(M_PI / 2, sinp) * 57.29578f;
  else
    angles.pitch = asin(sinp) * 57.29578f;
  angles.pitch = -1.0f * angles.pitch;
  // Yaw (z-axis rotation)
  float siny_cosp = 2 * (q.q0 * q.q3 + q.q1 * q.q2);
  float cosy_cosp = 1 - 2 * (q.q2 * q.q2 + q.q3 * q.q3);
  angles.yaw = atan2(siny_cosp, cosy_cosp) * 57.29578f;

  return angles;
}
