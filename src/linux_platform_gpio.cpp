#include <arl_hw/linux_platform_gpio.h>

LinuxPlatform_GPIO::LinuxPlatform_GPIO() {
}

LinuxPlatform_GPIO::~LinuxPlatform_GPIO() {}

bool LinuxPlatform_GPIO::initGPIO(int gpio_num) {
  return true;
}

bool LinuxPlatform_GPIO::deinitGPIO(int gpio_num) {
  return true;
}

bool LinuxPlatform_GPIO::setMode(int gpio_num, LinuxPlatform_GPIO::gpio_mode mode) {
  return true;
}

bool LinuxPlatform_GPIO::setState(int gpio_num, LinuxPlatform_GPIO::gpio_state state) {
  return true;
}

LinuxPlatform_GPIO::gpio_state LinuxPlatform_GPIO::readState(int gpio_num) {
  return LinuxPlatform_GPIO::gpio_state::OFF;
}