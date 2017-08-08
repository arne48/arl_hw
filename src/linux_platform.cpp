#include <arl_hw/linux_platform.h>
#include <ros/ros.h>

LinuxPlatform::LinuxPlatform() {

  _gpio = new LinuxPlatform_GPIO();
  _spi = new LinuxPlatform_SPI(_gpio);

  _dac = new AD5360(_spi);
  _adc = new AD7616(_spi);
  _lcell = new AD7730(_spi);


  //DAC-Latch
  _gpio->initGPIO(_dac_latch);
  _gpio->setMode(_dac_latch, LinuxPlatform_GPIO::gpio_mode::OUTPUT);
  _gpio->setState(_dac_latch, LinuxPlatform_GPIO::gpio_state::OFF);

  //ADC-Convst
  _gpio->initGPIO(_adc_conversion_start);
  _gpio->setMode(_adc_conversion_start, LinuxPlatform_GPIO::gpio_mode::OUTPUT);
  _gpio->setState(_adc_conversion_start, LinuxPlatform_GPIO::gpio_state::OFF);

}

LinuxPlatform::~LinuxPlatform() {
  delete _lcell;
  delete _adc;
  delete _dac;
  delete _gpio;
  delete _spi;
}

bool LinuxPlatform::read(std::vector<arl_datatypes::muscle_status_data_t> &status, std::vector<std::pair<int, int> > pressure_controllers,
                       std::vector<std::pair<int, int> > tension_controllers) {

  return true;
}

bool LinuxPlatform::write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec) {

  return true;
}

bool LinuxPlatform::initialize(std::vector<std::pair<int, int> > pressure_controllers,
                             std::vector<std::pair<int, int> > tension_controllers) {

  return true;
}

bool LinuxPlatform::close() {
  return true;
}

void LinuxPlatform::emergency_stop(std::pair<int, int> muscle) {
}

void LinuxPlatform::reset_muscle(std::pair<int, int> muscle) {
}