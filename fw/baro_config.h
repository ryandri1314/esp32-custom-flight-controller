#ifndef __BARO_CONFIG_T_H__
#define __BARO_CONFIG_T_H__

class Baro_config_t {
public:
  void init();
  bool read_raw_data(float *_temperature, float *_pressure);
  bool is_data_ready();
};

#endif