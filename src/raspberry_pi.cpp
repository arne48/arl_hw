#include <arl_hw/raspberry_pi.h>

RaspberryPi::RaspberryPi() {
}

RaspberryPi::~RaspberryPi() {
}

arl_datatypes::device_data_t RaspberryPi::read() {

  delayMicroseconds(100);
  digitalWrite(0, HIGH);

  arl_datatypes::device_data_t ret;

  return ret;
}

bool RaspberryPi::write(arl_datatypes::device_command_t command) {

  delayMicroseconds(100);
  digitalWrite(0, LOW);

  return true;
}

bool RaspberryPi::initialize() {

  wiringPiSetup();
  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);

  return true;
}

bool RaspberryPi::close() {
  return true;
}




