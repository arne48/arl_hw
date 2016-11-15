#include <arl_hw/dummy.h>

Dummy::Dummy() {
}

Dummy::~Dummy() {
}

arl_datatypes::device_data Dummy::read() {
  arl_datatypes::device_data ret;
  return ret;
}

bool Dummy::write(arl_datatypes::device_command) {
  return true;
}

bool Dummy::initialize() {
  return true;
}

bool Dummy::close() {
  return true;
}




