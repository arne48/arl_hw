#include <arl_hw/tinkerboard.h>
#include <ros/ros.h>

TinkerBoard::TinkerBoard() {

  if (bcm2835_init() != 1) {
    ROS_ERROR("bcm2835_init failed.");
  }

  _gpio = new TinkerBoard_GPIO();
  _spi = new TinkerBoard_SPI(_gpio);
  _dac = new AD5360(_spi);
  _adc = new AD7616(_spi, _gpio, _adc_conversion_port);
  _lcell = new AD7730(_spi);

  //Chip-Selects
  for(uint8_t idx = 0; idx < 16; idx++) {
    bcm2835_gpio_fsel(_gpios[idx], BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(_gpios[idx], HIGH);
  }

  //DAC-Latch
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_18, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_18, LOW);

  //ADC-Convst
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_16, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_16, HIGH);

}

TinkerBoard::~TinkerBoard() {
  delete _adc;
  delete _dac;
  delete _spi;
  bcm2835_close();
}

bool TinkerBoard::read(std::vector<arl_datatypes::muscle_status_data_t> &status, std::vector<std::pair<int, int> > pressure_controllers,
                       std::vector<std::pair<int, int> > tension_controllers) {

  std::map<int, uint16_t[16]> pressure_storage;
  for (auto const &entity : _pressure_ports) {
    for (int channel : entity.second) {
      uint32_t read_data = _adc->getMeasurementPair(_gpios[entity.first], channel);
      pressure_storage[entity.first][channel] = (uint16_t) ((read_data & 0xFFFF0000) >> 16);
      pressure_storage[entity.first][channel + 8] = (uint16_t) (read_data & 0x0000FFFF);
    }
  }


  std::map<int, uint32_t[16]> tension_storage;
  for (auto const &entity : _tension_ports) {
    _lcell->readData(_gpios[entity.first], _lcell_buffer);
    for (int channel : entity.second) {

      tension_storage[entity.first][channel] =
        _lcell_buffer[(3 * channel)] << 16 | _lcell_buffer[(3 * channel) + 1] << 8 | _lcell_buffer[(3 * channel) + 2];
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

bool TinkerBoard::write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec) {
  for (arl_datatypes::muscle_command_data_t command : command_vec) {
    _dac->setNormalized(_gpios[command.controller_port_activation.first], (uint8_t) command.controller_port_activation.second / (uint8_t) 8,
                        (uint8_t) command.controller_port_activation.second % (uint8_t) 8,
                        command.activation);
  }
  return true;
}

bool TinkerBoard::initialize(std::vector<std::pair<int, int> > pressure_controllers,
                             std::vector<std::pair<int, int> > tension_controllers) {

  //Determine which channels need to be read in order to not have to read whole controller
  for (std::pair<int, int> controller : pressure_controllers) {
    _pressure_ports[controller.first].insert(controller.second % 8);
  }


  for (std::pair<int, int> controller : tension_controllers) {
    _tension_ports[controller.first].insert(controller.second);
  }

  return true;
}

bool TinkerBoard::close() {
  return true;
}

void TinkerBoard::emergency_stop(std::pair<int, int> muscle) {
  _dac->setVoltage(_gpios[muscle.first], (uint8_t) muscle.second / (uint8_t) 8, (uint8_t) muscle.second % (uint8_t) 8, BLOW_OFF_VOLTAGE);
}

void TinkerBoard::reset_muscle(std::pair<int, int> muscle) {
  _dac->reset(_gpios[muscle.first], (uint8_t) muscle.second / (uint8_t) 8, (uint8_t) muscle.second % (uint8_t) 8);
}
