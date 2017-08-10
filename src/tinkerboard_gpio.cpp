#include <arl_hw/tinkerboard_gpio.h>

TinkerBoard_GPIO::TinkerBoard_GPIO() {
}

TinkerBoard_GPIO::~TinkerBoard_GPIO() {}

void TinkerBoard_GPIO::init() {
}

void TinkerBoard_GPIO::deinit() {
}

void TinkerBoard_GPIO::set_mode(int gpio_address, Embedded_GPIO::gpio_mode mode) {
}

void TinkerBoard_GPIO::set_output(int gpio_num, Embedded_GPIO::gpio_state) {
}

Embedded_GPIO::gpio_state TinkerBoard_GPIO::read_input(int gpio_address){
  return Embedded_GPIO::gpio_state::OFF;
}