#include <arl_hw/dummy.h>

Dummy::Dummy() {
}

Dummy::~Dummy() {
}

arl_datatypes::device_data_t Dummy::read() {
  arl_datatypes::device_data_t ret;
  return ret;
}

bool Dummy::write(arl_datatypes::device_command_t) {
  return true;
}

bool Dummy::initialize() {
  return true;
}

bool Dummy::close() {
  return true;
}




