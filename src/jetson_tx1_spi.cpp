#include "arl_hw/jetson_tx1_spi.h"
#include <ros/ros.h>

JetsonTX1_SPI::JetsonTX1_SPI() {
}

JetsonTX1_SPI::JetsonTX1_SPI(JetsonTX1_GPIO *gpio) {
  _gpio = gpio;
}

JetsonTX1_SPI::~JetsonTX1_SPI() {
}

bool JetsonTX1_SPI::transferSPI(int cs, int data_len, char data[]) {

  //setting chip-select
  _gpio->setCSByMultiplexerAddress(cs);
  _gpio->setState(219, JetsonTX1_GPIO::gpio_state::OFF);

  //write


  //resetting chip-select
  _gpio->setState(219, JetsonTX1_GPIO::gpio_state::ON);
  return true;
}

void JetsonTX1_SPI::setSCLKDivider(int divider) {
}
