#ifndef __RX_DATA_H__
#define __RX_DATA_H__

#include "rx_config.h"

class RX_Data {
private:
  RX_Config_t _drv;
  uint16_t _data[NUMBER_OF_CHANNELS];
  uint16_t _raw_data[NUMBER_OF_CHANNELS];

public:
  void init();
  void update_raw_data();
  void convert_data();
  bool is_failsafe();
  bool is_lost_frame();

  uint16_t* get_data();
};

#endif