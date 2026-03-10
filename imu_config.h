#ifndef __IMU_CONFIG_T__
#define __IMU_CONFIG_T__

#define IMU_ADDRESS 0x68
#define PERFORM_CALIBRATION 

class Accel_data {
public:
  float _accel_x;
  float _accel_y;
  float _accel_z;
};

class Gyro_data {
public:
  float _gyro_x;
  float _gyro_y;
  float _gyro_z;
};

class Imu_config_t {
public:
  void init();
  void calib();
  bool read_raw_data(Accel_data *_accel, Gyro_data *_gyro, float *tempurature);
};

#endif 