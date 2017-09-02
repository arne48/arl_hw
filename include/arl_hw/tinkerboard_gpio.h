#ifndef ARL_HW_TINKERBOARD_GPIO_H
#define ARL_HW_TINKERBOARD_GPIO_H

#include <arl_hw/embedded_gpio.h>
#include <tinkerboard_io.h>

class TinkerBoard_GPIO : public Embedded_GPIO {
public:

  TinkerBoard_GPIO();

  ~TinkerBoard_GPIO();

  virtual void init(int gpio_address);

  virtual void deinit(int gpio_address);

  virtual void set_mode(int gpio_address, Embedded_GPIO::gpio_mode mode);

  virtual void set_output(int gpio_address, Embedded_GPIO::gpio_state state);

  virtual Embedded_GPIO::gpio_state read_input(int gpio_address);

};


#endif //ARL_HW_TINKERBOARD_GPIO_H
