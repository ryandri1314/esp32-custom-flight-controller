#include "imu_config.h"
#include "FastIMU.h"
#include <Wire.h>

MPU6500 IMU;

calData calib_val = { 0 };
AccelData accelData;
GyroData gyroData;

void Imu_config_t::init() {
  Wire.begin();
  int err = IMU.init(calib_val, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

void Imu_config_t::calib() {
  Serial.println("Start calibration...");

  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);

  IMU.calibrateAccelGyro(&calib_val);
  delay(5000);
  IMU.init(calib_val, IMU_ADDRESS);
}

bool Imu_config_t::read_raw_data(Accel_data *_accel, Gyro_data *_gyro, float *tempurature) {
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  _accel->_accel_x = accelData.accelX;
  _accel->_accel_y = accelData.accelY;
  _accel->_accel_z = accelData.accelZ;

  _gyro->_gyro_x = gyroData.gyroX;
  _gyro->_gyro_y = gyroData.gyroY;
  _gyro->_gyro_z = gyroData.gyroZ;

	*tempurature = IMU.getTemp();
  return true;
}