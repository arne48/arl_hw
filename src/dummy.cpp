#include <arl_hw/dummy.h>
#include <map>
#include <cstring>

Dummy::Dummy() {
}

Dummy::~Dummy() {
}

bool Dummy::read(std::vector<arl_datatypes::muscle_status_data_t> &status, std::vector<std::pair<int, int> > pressure_controllers,
                 std::vector<std::pair<int, int> > tension_controllers) {

  //Determine which channels need to be read in order to not have to read whole controller
  std::map<int, std::set<int>> pressure_ports;
  for (std::pair<int, int> controller : pressure_controllers) {
    pressure_ports[controller.first].insert(controller.second % 8);
  }

  std::map<int, uint32_t[16]> storage;
  for (auto const &entity : pressure_ports) {
    for (int channel : entity.second) {
      uint32_t read_data = 0b10000000000000001000000000000000;
      storage[entity.first][channel] = (uint16_t) ((read_data & 0xFFFF0000) >> 16);
      storage[entity.first][channel + 8] = (uint16_t) (read_data & 0x0000FFFF);
    }
  }

  status.clear();
  for (unsigned int i = 0; i < pressure_controllers.size(); i++) {
    status.push_back(
      //force wrong order to test correct checking in calling function (robot->read())
      {pressure_controllers[(i-1)%pressure_controllers.size()], tension_controllers[(i-1)%pressure_controllers.size()], double(i), double(i)});
  }

  return true;
}

bool Dummy::write(std::vector<arl_datatypes::muscle_command_data_t> &command) {
  return true;
}

bool Dummy::initialize() {
  return true;
}

bool Dummy::close() {
  return true;
}

void Dummy::emergency_stop(std::pair<int, int> muscle) {

}

void Dummy::reset_muscle(std::pair<int, int> muscle) {

}




