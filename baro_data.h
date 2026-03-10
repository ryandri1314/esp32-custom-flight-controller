#ifndef __BARO_DATA_H__
#define __BARO_DATA_H__

#include "baro_config.h"

#define CALIB_TIMES 100

class Baro_Data {
private:
  Baro_config_t _drv;

  float _offset;
  float _temperature;
  float _pressure;
  float _altitude;

public:
  void init();
  void calibrate();
  void update_data_val();

  float get_pressure() const;
  float get_temperature() const;
  float get_altitude() const;
  float get_offset() const;
  float compute_altitude(float pressure_hpa) const;
};

#endif