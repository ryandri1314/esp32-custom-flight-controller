#include "imu_data.h"

void Imu_Data::init() {
  this->_drv.init();
}

void Imu_Data::calibration() {
  this->_drv.calib();
}

void Imu_Data::update_data_val() {
  this->_drv.read_raw_data(&this->_accel, &this->_gyro, &this->_tempurature);
}

void Imu_Data::get_accel(float *ax, float *ay, float *az) const {
  *ax = this->_accel._accel_x;
  *ay = this->_accel._accel_y;
  *az = this->_accel._accel_z;
}

void Imu_Data::get_gyro(float *gx, float *gy, float *gz) const {
  *gx = this->_gyro._gyro_x * 0.01745329251f;
  *gy = this->_gyro._gyro_y * 0.01745329251f;
  *gz = this->_gyro._gyro_z * 0.01745329251f;
}

void Imu_Data::get_tempurature(float *tempurature) const {
  *tempurature = this->_tempurature;
}
