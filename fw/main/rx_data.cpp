#include "rx_data.h"

RX_Config_t rx_config;

void RX_Data::init() {
  this->_drv = rx_config;
  memset(this->_raw_data, 0, sizeof(this->_raw_data));
  memset(this->_data, 0, sizeof(this->_data));
  this->_drv.init();
}

void RX_Data::update_raw_data() {
  bool successed = this->_drv.read_raw_data(this->_raw_data);
  if (!successed) {
    Serial.println("Fail to read!");
  }
}

void RX_Data::convert_data() {
  for (uint8_t i = 0; i < NUMBER_OF_CHANNELS; i++) {
    this->_data[i] = map(this->_raw_data[i], 192, 1792, 1000, 2000);
  }
}

bool RX_Data::is_failsafe() {
  return this->_drv.is_failsafe();
}

bool RX_Data::is_lost_frame() {
  return this->_drv.is_lost_frame();
}

uint16_t* RX_Data::get_data() {
  return this->_data;
}