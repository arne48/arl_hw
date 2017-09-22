#ifndef ARL_HW_LINUX_PLATFORM_GPIO_H
#define ARL_HW_LINUX_PLATFORM_GPIO_H

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <poll.h>

#include <arl_hw/embedded_gpio.h>

#define GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 256

class LinuxPlatform_GPIO : public Embedded_GPIO {
public:

  LinuxPlatform_GPIO();

  ~LinuxPlatform_GPIO();

  virtual void init(int gpio_address);

  virtual void deinit(int gpio_address);

  virtual void set_mode(int gpio_address, Embedded_GPIO::gpio_mode mode);

  virtual void set_output(int gpio_address, Embedded_GPIO::gpio_state state);

  virtual Embedded_GPIO::gpio_state read_input(int gpio_address);

};


#endif //ARL_HW_LINUX_PLATFORM_GPIO_H
