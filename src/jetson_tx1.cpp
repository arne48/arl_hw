#include "arl_hw/jetson_tx1.h"
#include <ros/ros.h>

JetsonTX1::JetsonTX1() {

  _gpio = new JetsonTX1_GPIO();
  _spi = new JetsonTX1_SPI(_gpio);
  _spi->setMultiplexerBusAddresses(_multiplexer_address_bus, _address_length, _enable, _latch);

  _dac = new AD5360(_spi);
  _adc = new AD7616(_spi);
  _lcell = new AD7730(_spi);

  //Multiplexer address bus
  for(int i=0; i < 4; i++){
    _gpio->initGPIO(_multiplexer_address_bus[i]);
    _gpio->setMode(_multiplexer_address_bus[i], JetsonTX1_GPIO::gpio_mode::OUTPUT);
    _gpio->setState(_multiplexer_address_bus[i], JetsonTX1_GPIO::gpio_state::ON);
  }

  //DAC-Latch
  _gpio->initGPIO(_dac_latch);
  _gpio->setMode(_dac_latch, JetsonTX1_GPIO::gpio_mode::OUTPUT);
  _gpio->setState(_dac_latch, JetsonTX1_GPIO::gpio_state::OFF);

  //ADC-Convst
  _gpio->initGPIO(_adc_conversion_start);
  _gpio->setMode(_adc_conversion_start, JetsonTX1_GPIO::gpio_mode::OUTPUT);
  _gpio->setState(_adc_conversion_start, JetsonTX1_GPIO::gpio_state::OFF);

}

JetsonTX1::~JetsonTX1() {
  delete _lcell;
  delete _adc;
  delete _dac;
  delete _gpio;
  delete _spi;
}

bool JetsonTX1::read(std::vector<arl_datatypes::muscle_status_data_t> &status, std::vector<std::pair<int, int> > pressure_controllers,
                       std::vector<std::pair<int, int> > tension_controllers) {

  std::map<int, uint16_t[16]> pressure_storage;
  for (auto const &entity : _pressure_ports) {
    for (int channel : entity.second) {
      uint32_t read_data = _adc->getMeasurementPair(_multiplexer_mapping[entity.first], channel);
      pressure_storage[entity.first][channel] = (uint16_t) ((read_data & 0xFFFF0000) >> 16);
      pressure_storage[entity.first][channel + 8] = (uint16_t) (read_data & 0x0000FFFF);
    }
  }

  std::map<int, uint16_t[16]> tension_storage;
  for (auto const &entity : _tension_ports) {
    _lcell->readData(_multiplexer_mapping[entity.first], _lcell_buffer);
    for (int channel : entity.second) {
      tension_storage[entity.first][channel] = _lcell_buffer[ 3 * channel];
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

bool JetsonTX1::write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec) {
  for (arl_datatypes::muscle_command_data_t command : command_vec) {
    _dac->setNormalized(_multiplexer_mapping[command.controller_port_activation.first], (uint8_t) command.controller_port_activation.second / (uint8_t) 8,
                        (uint8_t) command.controller_port_activation.second % (uint8_t) 8,
                        command.activation);
  }
  return true;
}

bool JetsonTX1::initialize(std::vector<std::pair<int, int> > pressure_controllers,
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

bool JetsonTX1::close() {
  return true;
}

void JetsonTX1::emergency_stop(std::pair<int, int> muscle) {
  _dac->setVoltage(_multiplexer_mapping[muscle.first], (uint8_t) muscle.second / (uint8_t) 8, (uint8_t) muscle.second % (uint8_t) 8, BLOW_OFF);
}

void JetsonTX1::reset_muscle(std::pair<int, int> muscle) {
  _dac->reset(_multiplexer_mapping[muscle.first], (uint8_t) muscle.second / (uint8_t) 8, (uint8_t) muscle.second % (uint8_t) 8);
}