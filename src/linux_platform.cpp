#include <arl_hw/linux_platform.h>
#include <ros/ros.h>

LinuxPlatform::LinuxPlatform() {

//  _gpio = new LinuxPlatform_GPIO();
//  _gpio->init();
//
//  _spi = new LinuxPlatform_SPI(_gpio);

  //_dac = new AD5360(_spi);
  //_adc = new AD7616(_spi, _gpio, _adc_conversion_port);
  //_lcell = new AD7730(_spi);

}

LinuxPlatform::~LinuxPlatform() {
//  delete _lcell;
//  delete _adc;
//  delete _dac;
//  delete _gpio;
//  delete _spi;
//  _gpio->deinit();
}

bool LinuxPlatform::read(std::vector<arl_datatypes::muscle_status_data_t> &status, std::vector<std::pair<int, int> > pressure_controllers,
                       std::vector<std::pair<int, int> > tension_controllers) {
  std::map<int, std::set<int>> pressure_ports;
  for (std::pair<int, int> controller : pressure_controllers) {
    pressure_ports[controller.first].insert(controller.second % 8);
  }
  std::map<int, std::set<int>> tension_ports;
  for (std::pair<int, int> controller : tension_controllers) {
    tension_ports[controller.first].insert(controller.second);
  }

  std::map<int, uint32_t[16]> storage;
  for (auto const &entity : pressure_ports) {
    for (int channel : entity.second) {
      uint32_t read_data = 0b10000000000000001000000000000000;
      storage[entity.first][channel] = (uint16_t) ((read_data & 0xFFFF0000) >> 16);
      storage[entity.first][channel + 8] = (uint16_t) (read_data & 0x0000FFFF);
    }
  }

  std::map<int, uint16_t[16]> tension_storage;
  uint16_t _lcell_buffer[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  for (auto const &entity : tension_ports) {
    for (int channel : entity.second) {
      tension_storage[entity.first][channel] = _lcell_buffer[channel];
    }
  }

  status.clear();
  for (unsigned int i = 0; i < pressure_controllers.size(); i++) {
    status.push_back(
        //force wrong order to test correct checking in calling function (robot->read())
        {pressure_controllers[(i - 1) % pressure_controllers.size()], tension_controllers[(i - 1) % pressure_controllers.size()], i,
         tension_storage[tension_controllers[(i - 1) % pressure_controllers.size()].first][tension_controllers[(i - 1) % pressure_controllers.size()].second]});
    //{pressure_controllers[i], tension_controllers[i], i,
    //tension_storage[tension_controllers[i].first][tension_controllers[i].second]});
  }

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
