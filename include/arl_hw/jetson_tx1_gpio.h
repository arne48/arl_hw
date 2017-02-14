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

  /**
   * Using assigned numbers of "pinctrl-tegra210.c" this call
   * function exports a pin as gpio using sysfs
   * @param gpio_num number of gpio port to initialize
   * @return success
   */
  bool initGPIO(int gpio_num);

  /**
   * Using assigned numbers of "pinctrl-tegra210.c" this call
   * function "unexports" a pin as gpio using sysfs
   * @param gpio_num number of gpio port to de-initialize
   * @return success
   */
  bool deinitGPIO(int gpio_num);

  /**
   * Once a gpio is exported the mode of it can be set
   * either to INPUT or OUTPUT
   * @param gpio_num number of gpio port set the mode of
   * @param mode mode to set gpio port to
   * @return success
   */
  bool setMode(int gpio_num, gpio_mode mode);

  /**
   * If a gpio is exported as an OUTPUT it's state can be set
   * either to ON or OFF (non-inverse logic ON means HIGH)
   * @param gpio_num number of gpio port set the state of
   * @param state state to set gpio port to
   * @return success
   */
  bool setState(int gpio_num, gpio_state state);

  /**
   * Due to the lack of gpios on the Jetson TX1 carrier board
   * an output multiplexer is used and utilized by this function.
   * @param address the output which will be set
   * @param bus_addresses a pointer to an array of gpio numbers representing the address LSB first
   * @param address_length length of the address
   * @param enable gpio number connected to the multiplexer's enable signal
   * @param latch gpio number connected to the multiplexer's latch signal
   * @return success
   */
  bool setCSByMultiplexerAddress(int address, int *bus_addresses, int address_length, int enable, int latch);

  /**
   * To reset all outputs to inverse-logic LOW (HIGH SIGNAL) the enable signal needs to be set
   * @param enable gpio number connected to the multiplexer's enable signal
   * @return
   */
  bool resetCSByMultiplexer(int enable);

  /**
   * This function reads the input state of a gpio which was set as an input
   * @param gpio_num number of gpio port to read input value from
   * @return
   */
  gpio_state readState(int gpio_num);

};


#endif //ARL_HW_JETSON_TX1_GPIO_H
