#include <arl_hw/raspberry_pi_gpio.h>

RaspberryPi_GPIO::RaspberryPi_GPIO() {
}

RaspberryPi_GPIO::~RaspberryPi_GPIO() {}

void RaspberryPi_GPIO::init() {
}

void RaspberryPi_GPIO::deinit() {
}

void RaspberryPi_GPIO::set_mode(int gpio_address, Embedded_GPIO::gpio_mode mode) {
  if(mode == Embedded_GPIO::gpio_mode::INPUT) {
    bcm2835_gpio_fsel((uint8_t)gpio_address, BCM2835_GPIO_FSEL_INPT);
  } else {
    bcm2835_gpio_fsel((uint8_t)gpio_address, BCM2835_GPIO_FSEL_OUTP);
  }
}

void RaspberryPi_GPIO::set_output(int gpio_address, Embedded_GPIO::gpio_state state) {
  if(state == Embedded_GPIO::gpio_state::ON) {
    bcm2835_gpio_write((uint8_t)gpio_address, HIGH);
  } else {
    bcm2835_gpio_write((uint8_t)gpio_address, LOW);
  }
}

Embedded_GPIO::gpio_state RaspberryPi_GPIO::read_input(int gpio_address){
  Embedded_GPIO::gpio_state ret = Embedded_GPIO::gpio_state::OFF;

  if(bcm2835_gpio_lev((uint8_t)gpio_address) == HIGH) {
    ret = Embedded_GPIO::gpio_state::ON;
  } else {
    ret = Embedded_GPIO::gpio_state::OFF;
  }

  return ret;
}