#include <arl_hw/linux_platform_gpio.h>

LinuxPlatform_GPIO::LinuxPlatform_GPIO() {
}

LinuxPlatform_GPIO::~LinuxPlatform_GPIO() {}

void LinuxPlatform_GPIO::init() {
}

void LinuxPlatform_GPIO::deinit() {
}

void LinuxPlatform_GPIO::set_mode(int gpio_address, Embedded_GPIO::gpio_mode mode) {
}

void LinuxPlatform_GPIO::set_output(int gpio_num, Embedded_GPIO::gpio_state) {
}

Embedded_GPIO::gpio_state LinuxPlatform_GPIO::read_input(int gpio_address){
  return Embedded_GPIO::gpio_state::OFF;
}