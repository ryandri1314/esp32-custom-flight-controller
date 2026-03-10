#include "rx_config.h"
#include "sbus.h"

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1, UART1_RX_PIN, UART1_TX_PIN, true);
/* SBUS data */
bfs::SbusData data;

void RX_Config_t::init() {
  Serial1.begin(100000, SERIAL_8E2, UART1_RX_PIN, UART1_TX_PIN);
  sbus_rx.Begin();
}

bool RX_Config_t::read_raw_data(uint16_t *_data) {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    for (uint8_t i = 0; i < NUMBER_OF_CHANNELS; ++i) {
      _data[i] = data.ch[i];
    }
    return true;
  }
  return false;
}

bool RX_Config_t::is_failsafe() {
  return data.failsafe;
}

bool RX_Config_t::is_lost_frame() {
  return data.lost_frame;
}
