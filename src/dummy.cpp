#include <arl_hw/dummy.h>

Dummy::Dummy() {
}

Dummy::~Dummy() {
}

bool Dummy::read(std::vector<arl_datatypes::muscle_status_data_t> &status) {
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

void Dummy::emergency_halt(std::pair<int, int> muscle) {

}

void Dummy::reset_muscle(std::pair<int, int> muscle) {

}




