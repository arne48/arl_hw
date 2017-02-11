#include <arl_hw/raspberry_pi.h>
#include <ros/ros.h>

RaspberryPi::RaspberryPi() {

  if (!bcm2835_init()) {
    ROS_ERROR("bcm2835_init failed.");
  }

  _spi = new RaspberryPi_SPI();
  _dac = new AD5360(_spi);
  _adc = new AD7616(_spi);
  _lcell = new AD7730(_spi);

  //Chip-Selects
  bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_26, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_32, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_36, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_07, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_11, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_13, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_15, BCM2835_GPIO_FSEL_OUTP);

  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_31, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_33, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_35, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_37, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_03, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_05, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_08, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_10, BCM2835_GPIO_FSEL_OUTP);

  bcm2835_gpio_write(RPI_GPIO_P1_24, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_26, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_32, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_36, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_07, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_11, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_13, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_15, HIGH);

  bcm2835_gpio_write(RPI_V2_GPIO_P1_31, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_33, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_35, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_37, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_03, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_05, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_08, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_10, HIGH);

  //DAC-Latch
  bcm2835_gpio_fsel(RPI_GPIO_P1_18, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(RPI_GPIO_P1_18, LOW);

  //ADC-Convst
  bcm2835_gpio_fsel(RPI_GPIO_P1_16, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(RPI_GPIO_P1_16, HIGH);

}

RaspberryPi::~RaspberryPi() {
  delete _adc;
  delete _dac;
  delete _spi;
  bcm2835_close();
}

bool RaspberryPi::read(std::vector<arl_datatypes::muscle_status_data_t> &status, std::vector<std::pair<int, int> > pressure_controllers,
                       std::vector<std::pair<int, int> > tension_controllers) {

  std::map<int, uint16_t[16]> pressure_storage;
  for (auto const &entity : _pressure_ports) {
    for (int channel : entity.second) {
      uint32_t read_data = _adc->getMeasurementPair(_gpios[entity.first], channel);
      pressure_storage[entity.first][channel] = (uint16_t) ((read_data & 0xFFFF0000) >> 16);
      pressure_storage[entity.first][channel + 8] = (uint16_t) (read_data & 0x0000FFFF);
    }
  }

  //FIXME
  std::map<int, uint16_t[16]> tension_storage;
  for (auto const &entity : _tension_ports) {
    _lcell->readData(_gpios[entity.first], _lcell_buffer);
    for (int channel : entity.second) {
      tension_storage[entity.first][channel] = (uint16_t) 0 | _lcell_buffer[channel * 2] << 8 | _lcell_buffer[(channel * 2) + 1];
    }
  }

  status.clear();
  for (unsigned int i = 0; i < pressure_controllers.size(); i++) {
    status.push_back({pressure_controllers[i], tension_controllers[i],
                      pressure_storage[pressure_controllers[i].first][pressure_controllers[i].second],
                      tension_storage[tension_controllers[i].first][tension_controllers[i].second]});
  }

  return true;
}

bool RaspberryPi::write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec) {
  for (arl_datatypes::muscle_command_data_t command : command_vec) {
    _dac->setNormalized(_gpios[command.controller_port_activation.first], (uint8_t) command.controller_port_activation.second / (uint8_t) 8,
                        (uint8_t) command.controller_port_activation.second % (uint8_t) 8,
                        command.activation);
  }
  return true;
}

bool RaspberryPi::initialize(std::vector<std::pair<int, int> > pressure_controllers,
                             std::vector<std::pair<int, int> > tension_controllers) {

  //Determine which channels need to be read in order to not have to read whole controller
  for (std::pair<int, int> controller : pressure_controllers) {
    _pressure_ports[controller.first].insert(controller.second % 8);
  }


  for (std::pair<int, int> controller : tension_controllers) {
    _tension_ports[controller.first].insert(controller.second);
  }

  for (auto const &entity : _tension_ports) {
    uint16_t mask = 0;
    for (int channel : entity.second) {
      if (channel < 1) {
        mask = mask | (uint16_t) 1;
      } else {
        mask = mask | ((uint16_t) 1 << channel);
      }
    }
    _lcell->setActiveChannelsByMask(_gpios[entity.first], mask);
  }

  return true;
}

bool RaspberryPi::close() {
  return true;
}

void RaspberryPi::emergency_stop(std::pair<int, int> muscle) {
  _dac->setVoltage(_gpios[muscle.first], (uint8_t) muscle.second / (uint8_t) 8, (uint8_t) muscle.second % (uint8_t) 8, BLOW_OFF);
}

void RaspberryPi::reset_muscle(std::pair<int, int> muscle) {
  _dac->reset(_gpios[muscle.first], (uint8_t) muscle.second / (uint8_t) 8, (uint8_t) muscle.second % (uint8_t) 8);
}





