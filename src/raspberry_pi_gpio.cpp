#include <arl_hw/raspberry_pi_gpio.h>

RaspberryPi_GPIO::RaspberryPi_GPIO() {
}

RaspberryPi_GPIO::~RaspberryPi_GPIO() {}

void RaspberryPi_GPIO::init() {
}

void RaspberryPi_GPIO::deinit() {
}

void RaspberryPi_GPIO::set_mode(int gpio_address, Embedded_GPIO::gpio_mode mode) {
}

void RaspberryPi_GPIO::set_output(int gpio_num, Embedded_GPIO::gpio_state) {
}

Embedded_GPIO::gpio_state RaspberryPi_GPIO::read_input(int gpio_address){
  return Embedded_GPIO::gpio_state::OFF;
}