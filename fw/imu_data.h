#ifndef __IMU_DATA_H__
#define __IMU_DATA_H__

#include "imu_config.h"
#include <Arduino.h>

class Imu_Data {
private:
  Imu_config_t _drv;

  Gyro_data _gyro;
  Accel_data _accel;
  float _tempurature;

public:
  void init();
  void calibration();
  void update_data_val();
  void get_accel(float *ax, float *ay, float *az) const;
  void get_gyro(float *gx, float *gy, float *gz) const;
  void get_tempurature(float *tempurature) const;
};

#endif