#ifndef ARL_HW_LINUX_PLATFORM_GPIO_H
#define ARL_HW_LINUX_PLATFORM_GPIO_H

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <poll.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64

class LinuxPlatform_GPIO {
public:
  enum gpio_mode { INPUT = 0, OUTPUT = 1 };
  enum gpio_state { OFF = 0, ON = 1 };

  /**
   * Default Constructor
   */
  LinuxPlatform_GPIO();

  /**
   * Destructor
   */
  ~LinuxPlatform_GPIO();

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
   * This function reads the input state of a gpio which was set as an input
   * @param gpio_num number of gpio port to read input value from
   * @return
   */
  gpio_state readState(int gpio_num);

};


#endif //ARL_HW_LINUX_PLATFORM_GPIO_H
