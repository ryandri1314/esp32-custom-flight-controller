#include "baro_data.h"
#include "baro_config.h"
#include <Arduino.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Baro_config_t baro_config;

/*  Baro_Data Functions  */

void Baro_Data::init() {
  this->_drv = baro_config;
  this->_pressure = 0;
  this->_temperature = 0;
  this->_offset = 0;
  this->_altitude = 0;
  this->_drv.init();
}

void Baro_Data::calibrate() {
  float temp = 0;
  for (uint16_t i = 0; i < CALIB_TIMES; i++) {
    this->update_data_val();
    temp += this->compute_altitude(this->_pressure);
    // Serial.print("Alt: ");
    // Serial.println(this->compute_altitude(this->_pressure));
    // Serial.print("Temp: ");
    // Serial.println(temp);
    delay(20);
  }
  this->_offset = temp / CALIB_TIMES;
}

void Baro_Data::update_data_val() {
  this->_drv.read_raw_data(&this->_temperature, &this->_pressure);
}

float Baro_Data::get_pressure() const {
  return this->_pressure;
}

float Baro_Data::get_temperature() const {
  return this->_temperature;
}

float Baro_Data::compute_altitude(float pressure_hpa) const {
  return 44330.0f * (1.0f - pow(pressure_hpa / SEALEVELPRESSURE_HPA, 0.1903f));
}

float Baro_Data::get_altitude() const {
  return (this->compute_altitude(this->_pressure) - this->_offset);
}

float Baro_Data::get_offset() const {
  return this->_offset;
}




