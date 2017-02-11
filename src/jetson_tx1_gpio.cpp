#include "arl_hw/jetson_tx1_gpio.h"

JetsonTX1_GPIO::JetsonTX1_GPIO() {
}

JetsonTX1_GPIO::~JetsonTX1_GPIO() {}

bool JetsonTX1_GPIO::initGPIO(int gpio_num) {
  char commandBuffer[MAX_BUF];
  int fileDescriptor = open(SYSFS_GPIO_DIR "/export", O_WRONLY);

  if (fileDescriptor < 0) {
    return false;
  }

  int length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", gpio_num);
  if (write(fileDescriptor, commandBuffer, length) != length) {
    return false;
  }

  close(fileDescriptor);
  return true;
}

bool JetsonTX1_GPIO::deinitGPIO(int gpio_num) {
  char commandBuffer[MAX_BUF];

  int fileDescriptor = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
  if (fileDescriptor < 0) {
    return false;
  }

  int length = snprintf(commandBuffer, sizeof(commandBuffer), "%d", gpio_num);
  if (write(fileDescriptor, commandBuffer, length) != length) {
    return false;
  }

  close(fileDescriptor);
  return 0;
}

bool JetsonTX1_GPIO::setMode(int gpio_num, JetsonTX1_GPIO::gpio_mode mode) {
  char commandBuffer[MAX_BUF];

  snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio_num);

  int fileDescriptor = open(commandBuffer, O_WRONLY);
  if (fileDescriptor < 0) {
    return false;
  }

  if (mode == JetsonTX1_GPIO::gpio_mode::OUTPUT) {
    if (write(fileDescriptor, "out", 4) != 4) {
      return false;
    }
  } else {
    if (write(fileDescriptor, "in", 3) != 3) {
      return false;
    }
  }

  close(fileDescriptor);
  return true;
}

bool JetsonTX1_GPIO::setState(int gpio_num, JetsonTX1_GPIO::gpio_state state) {
  char commandBuffer[MAX_BUF];

  snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio_num);

  int fileDescriptor = open(commandBuffer, O_WRONLY);
  if (fileDescriptor < 0) {
    return false;
  }

  if (state == JetsonTX1_GPIO::gpio_state::ON) {
    if (write(fileDescriptor, "1", 2) != 2) {
      return false;
    }
  } else {
    if (write(fileDescriptor, "0", 2) != 2) {
      return false;
    }
  }

  close(fileDescriptor);
  return true;
}

JetsonTX1_GPIO::gpio_state JetsonTX1_GPIO::readState(int gpio_num) {
  char commandBuffer[MAX_BUF];
  snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value", gpio_num);

  int fileDescriptor = open(commandBuffer, O_RDONLY);
  if (fileDescriptor < 0) {
    return JetsonTX1_GPIO::gpio_state::OFF;
  }

  char ch;
  if (read(fileDescriptor, &ch, 1) != 1) {
    return JetsonTX1_GPIO::gpio_state::OFF;
  }

  if (ch != '0') {
    close(fileDescriptor);
    return JetsonTX1_GPIO::gpio_state::ON;
  } else {
    close(fileDescriptor);
    return JetsonTX1_GPIO::gpio_state::OFF;
  }

}

bool JetsonTX1_GPIO::setCSByMultiplexerAddress(int address, int *bus_addresses, int address_length, int enable, int latch){
  setState(latch, JetsonTX1_GPIO::gpio_state::ON);

  for(int i=0; i < address_length; i++){
    if(address & (1 << i)){
      setState(bus_addresses[0], JetsonTX1_GPIO::gpio_state::ON);
    } else {
      setState(bus_addresses[0], JetsonTX1_GPIO::gpio_state::OFF);
    }
  }

  setState(latch, JetsonTX1_GPIO::gpio_state::OFF);
  setState(enable, JetsonTX1_GPIO::gpio_state::OFF);

  return true;
}

bool JetsonTX1_GPIO::resetCSByMultiplexer(int enable){
  setState(enable, JetsonTX1_GPIO::gpio_state::ON);
  return true;
}
