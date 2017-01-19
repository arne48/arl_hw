#ifndef ARL_HW_JETSON_TX1_GPIO_H
#define ARL_HW_JETSON_TX1_GPIO_H

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <poll.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64

class JetsonTX1_GPIO {
public:
  enum gpio_mode { INPUT = 0, OUTPUT = 1 };
  enum gpio_state { OFF = 0, ON = 1 };

  /**
   * Default Constructor
   */
  JetsonTX1_GPIO();

  /**
   * Destructor
   */
  ~JetsonTX1_GPIO();

  bool initGPIO(int gpio_num);

  bool deinitGPIO(int gpio_num);

  bool setMode(int gpio_num, gpio_mode mode);

  bool setState(int gpio_num, gpio_state state);

  bool setCSByMultiplexerAddress(int address, int *bus_addresses, int address_length, int enable, int latch);

  bool resetCSByMultiplexer(int enable);

  gpio_state readState(int gpio_num);

};


#endif //ARL_HW_JETSON_TX1_GPIO_H
