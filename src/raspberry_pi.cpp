#include <arl_hw/raspberry_pi.h>

RaspberryPi::RaspberryPi() {
}

RaspberryPi::~RaspberryPi() {
}

arl_datatypes::device_data RaspberryPi::read() {

  delayMicroseconds(100);
  digitalWrite(0, HIGH);

  arl_datatypes::device_data ret;

  return ret;
}

bool RaspberryPi::write(arl_datatypes::device_command command) {

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




