#include <arl_hw/linux_platform.h>
#include <ros/ros.h>

LinuxPlatform::LinuxPlatform() {
  _gpio = new LinuxPlatform_GPIO();

  for(uint8_t idx=0; idx<16; idx++) {
    _gpio->init(_gpios[idx]);
    _gpio->set_mode(_gpios[idx], Embedded_GPIO::gpio_mode::OUTPUT);
    _gpio->set_output(_gpios[idx], Embedded_GPIO::gpio_state::ON);
  }
  _gpio->init(_adc_conversion_port);
  _gpio->init(_dac_latch_port);

  _gpio->set_mode(_dac_latch_port, Embedded_GPIO::gpio_mode::OUTPUT);
  _gpio->set_output(_dac_latch_port, Embedded_GPIO::gpio_state::OFF);

  _gpio->set_mode(_adc_conversion_port, Embedded_GPIO::gpio_mode::OUTPUT);
  _gpio->set_output(_adc_conversion_port, Embedded_GPIO::gpio_state::ON);

  _spi = new LinuxPlatform_SPI(_gpio);

  _dac = new AD5360(_spi);
  _adc = new AD7616(_spi, _gpio, _adc_conversion_port);
  _lcell = new AD7730(_spi);

}

LinuxPlatform::~LinuxPlatform() {
  for(uint8_t idx=0; idx<16; idx++) {
    _gpio->deinit(_gpios[idx]);
  }
  _gpio->deinit(_adc_conversion_port);
  _gpio->deinit(_dac_latch_port);

  delete _lcell;
  delete _adc;
  delete _dac;
  delete _gpio;
  delete _spi;
}

bool LinuxPlatform::read(std::vector<arl_datatypes::muscle_status_data_t> &status, std::vector<std::pair<int, int> > pressure_controllers,
                         std::vector<std::pair<int, int> > tension_controllers,
                         std::vector<arl_datatypes::analog_input_status_data_t> &analog_input_status,
                         std::vector<std::pair<int, int> > analog_inputs_controllers) {

  std::map<int, uint16_t[16]> analog_input_storage;
  for (auto const &entity : _analog_input_ports) {
    for (int channel : entity.second) {
      uint32_t read_data = _adc->getMeasurementPair(_gpios[entity.first], channel);
      analog_input_storage[entity.first][channel] = (uint16_t) ((read_data & 0xFFFF0000) >> 16);
      analog_input_storage[entity.first][channel + 8] = (uint16_t) (read_data & 0x0000FFFF);
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
    //V
    double voltage = (10.0 / 32768.0) * (int16_t)analog_input_storage[pressure_controllers[i].first][pressure_controllers[i].second];

    //MPa
    double pressure = (1.0 / 4.0) * (voltage - 1.0);
    status.push_back({pressure_controllers[i], tension_controllers[i],
                      pressure,
                      tension_storage[tension_controllers[i].first][tension_controllers[i].second]});
  }

  analog_input_status.clear();
  for (unsigned int i = 0; i < analog_inputs_controllers.size(); i++) {
    //V
    double voltage = (10.0 / 32768.0) * (int16_t)analog_input_storage[analog_inputs_controllers[i].first][analog_inputs_controllers[i].second];
    analog_input_status.push_back({analog_inputs_controllers[i],
                                   voltage});
  }

  return true;
}

bool LinuxPlatform::write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec) {
  for (arl_datatypes::muscle_command_data_t command : command_vec) {
    _dac->setNormalized(_gpios[command.controller_port_activation.first], (uint8_t) command.controller_port_activation.second / (uint8_t) 8,
                        (uint8_t) command.controller_port_activation.second % (uint8_t) 8,
                        command.activation);
  }
  return true;
}

bool LinuxPlatform::initialize(std::vector<std::pair<int, int> > pressure_controllers,
                               std::vector<std::pair<int, int> > tension_controllers,
                               std::vector<std::pair<int, int> > analog_inputs_controllers) {

  //Determine which channels need to be read in order to not have to read whole controller
  for (std::pair<int, int> controller : pressure_controllers) {
    _analog_input_ports[controller.first].insert(controller.second % 8);
  }

  for (std::pair<int, int> controller : analog_inputs_controllers) {
    _analog_input_ports[controller.first].insert(controller.second % 8);
  }

  for (std::pair<int, int> controller : tension_controllers) {
    _tension_ports[controller.first].insert(controller.second);
  }

  return true;
}

bool LinuxPlatform::close() {
  return true;
}

void LinuxPlatform::emergency_stop(std::pair<int, int> muscle) {
  _dac->setVoltage(_gpios[muscle.first], (uint8_t) muscle.second / (uint8_t) 8, (uint8_t) muscle.second % (uint8_t) 8, BLOW_OFF_VOLTAGE);
}

void LinuxPlatform::reset_muscle(std::pair<int, int> muscle) {
  _dac->reset(_gpios[muscle.first], (uint8_t) muscle.second / (uint8_t) 8, (uint8_t) muscle.second % (uint8_t) 8);
}
