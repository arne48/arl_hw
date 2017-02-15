#ifndef ARL_HW_EMBEDDED_GPIO_H
#define ARL_HW_EMBEDDED_GPIO_H

/**
 * Interface class for using the GPIO ports of an arbitrary platform
 * initialization needs to be done in a platform dependent fashion
 */
class Embedded_GPIO {
public:
  enum gpio_state { OFF = 0, ON = 1 };

  /**
   * Default Constructor
   */
  Embedded_GPIO() {};

  /**
   * Destructor
   */
  virtual ~Embedded_GPIO() {};

  virtual bool set_output(int gpio_address, gpio_state output_state) = 0;

  virtual gpio_state read_input(int gpio_address) = 0;
};

#endif //ARL_HW_EMBEDDED_GPIO_H
