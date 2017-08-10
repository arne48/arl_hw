#ifndef ARL_HW_RASPBERRY_PI_GPIO_H
#define ARL_HW_RASPBERRY_PI_GPIO_H

#include <arl_hw/embedded_gpio.h>

class RaspberryPi_GPIO : public Embedded_GPIO {
public:

  RaspberryPi_GPIO();

  ~RaspberryPi_GPIO();

  virtual void init();

  virtual void deinit();

  virtual void set_mode(int gpio_address, Embedded_GPIO::gpio_mode mode);

  virtual void set_output(int gpio_num, Embedded_GPIO::gpio_state);

  virtual Embedded_GPIO::gpio_state read_input(int gpio_address);

};


#endif //ARL_HW_RASPBERRY_PI_GPIO_H
