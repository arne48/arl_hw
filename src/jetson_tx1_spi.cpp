#include "arl_hw/jetson_tx1_spi.h"
#include <ros/ros.h>

JetsonTX1_SPI::JetsonTX1_SPI() {
  _enable = 0;
  _latch = 0;
  _address_length = 0;
}

JetsonTX1_SPI::JetsonTX1_SPI(JetsonTX1_GPIO *gpio) {
  _gpio = gpio;
}

JetsonTX1_SPI::~JetsonTX1_SPI() {
}

bool JetsonTX1_SPI::transferSPI(int cs, int data_len, char data[]) {

  //setting chip-select
  _gpio->setCSByMultiplexerAddress(cs, _multiplexer_address_bus, _address_length, _enable, _latch);

  //write
  //TODO SPI ACCESS

  //resetting chip-select
  _gpio->resetCSByMultiplexer(_enable);
  return true;
}

void JetsonTX1_SPI::setSCLKDivider(int divider) {
}

void JetsonTX1_SPI::setMultiplexerBusAddresses(int *bus_addresses, int address_length, int enable, int latch){
  _multiplexer_address_bus = bus_addresses;
  _address_length = address_length;
  _enable = enable;
  _latch = latch;
}
