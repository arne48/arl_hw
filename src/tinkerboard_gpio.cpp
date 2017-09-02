#include <arl_hw/tinkerboard_gpio.h>

TinkerBoard_GPIO::TinkerBoard_GPIO() {
}

TinkerBoard_GPIO::~TinkerBoard_GPIO() {}

void TinkerBoard_GPIO::init(int gpio_address) {
}

void TinkerBoard_GPIO::deinit(int gpio_address) {
}

void TinkerBoard_GPIO::set_mode(int gpio_address, Embedded_GPIO::gpio_mode mode) {
  if(mode == Embedded_GPIO::gpio_mode::OUTPUT) {
    tinkerboard_set_gpio_mode((uint32_t) gpio_address, IOMode::OUTPUT);
  } else if(mode == Embedded_GPIO::gpio_mode::INPUT) {
    tinkerboard_set_gpio_mode((uint32_t) gpio_address, IOMode::INPUT);
  }
}

void TinkerBoard_GPIO::set_output(int gpio_address, Embedded_GPIO::gpio_state state) {
  if(state == Embedded_GPIO::gpio_state::ON) {
    tinkerboard_set_gpio_state((uint32_t) gpio_address, IOState::HIGH);
  } else if(state == Embedded_GPIO::gpio_state::OFF) {
    tinkerboard_set_gpio_state((uint32_t) gpio_address, IOState::LOW);
  }
}

Embedded_GPIO::gpio_state TinkerBoard_GPIO::read_input(int gpio_address){
  enum IOState state = tinkerboard_get_gpio_state((uint32_t) gpio_address);

  if(state == IOState::HIGH) {
    return Embedded_GPIO::gpio_state::ON;
  } else {
    return Embedded_GPIO::gpio_state::OFF;
  }
}