#ifndef __RX_CONFIG_H__
#define __RX_CONFIG_H__

#include <Arduino.h>

#define NUMBER_OF_CHANNELS 10
#define UART1_RX_PIN  20
#define UART1_TX_PIN  21

class RX_Config_t {
public:
  void init();
  bool read_raw_data(uint16_t *_data);
  bool is_failsafe();
  bool is_lost_frame();
};

#endif