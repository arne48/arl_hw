#include <arl_hw/raspberry_pi.h>
#include <ros/ros.h>

RaspberryPi::RaspberryPi() {

  if (!bcm2835_init()) {
    ROS_ERROR("bcm2835_init failed.");
  }

  _spi = new RaspberryPi_SPI();
  _dac = new AD5360(_spi);


  //Chip-Selects
  bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_26, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_32, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_36, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_07, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_11, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_13, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_15, BCM2835_GPIO_FSEL_OUTP);


  bcm2835_gpio_write(RPI_GPIO_P1_24, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_26, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_32, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_36, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_07, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_11, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_13, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_15, HIGH);

  //Latch
  bcm2835_gpio_fsel(RPI_GPIO_P1_18, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(RPI_GPIO_P1_18, LOW);

}

RaspberryPi::~RaspberryPi() {
  delete _spi;
  bcm2835_close();
}

bool RaspberryPi::read(std::vector<arl_datatypes::muscle_status_data_t> &status) {
  return true;
}

bool RaspberryPi::write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec) {
  for (arl_datatypes::muscle_command_data_t command : command_vec) {
    _dac->setNormalized(gpios[command.controller_port_activation.first], (uint8_t) command.controller_port_activation.second / (uint8_t) 8,
                        (uint8_t) command.controller_port_activation.second % (uint8_t) 8,
                        command.activation);
  }
  return true;
}

bool RaspberryPi::initialize() {
  return true;
}

bool RaspberryPi::close() {
  return true;
}

void RaspberryPi::emergency_halt(std::pair<int, int> muscle) {
  _dac->setVoltage(gpios[muscle.first], (uint8_t) muscle.second / (uint8_t) 8, (uint8_t) muscle.second % (uint8_t) 8, BLOW_OFF);
}
void RaspberryPi::reset_muscle(std::pair<int, int> muscle) {
  _dac->reset(gpios[muscle.first], (uint8_t) muscle.second / (uint8_t) 8, (uint8_t) muscle.second % (uint8_t) 8);
}





