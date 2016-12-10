#include <arl_hw/raspberry_pi.h>

RaspberryPi::RaspberryPi() {
  wiringPiSetup();

  _spi = new RaspberryPi_SPI();
  _adc_vec.push_back(new AD5360(0,5,_spi));

}

RaspberryPi::~RaspberryPi() {
}

bool RaspberryPi::read(std::vector<arl_datatypes::muscle_status_data_t> &status) {
  _adc_vec[0]->setVoltage(BANKALL,CHALL,3.0);
  //_adc_vec[0]->latchDAC();
  return true;
}

bool RaspberryPi::write(std::vector<arl_datatypes::muscle_command_data_t> &command) {
  _adc_vec[0]->setVoltage(BANKALL,CHALL,1.0);
  //_adc_vec[0]->latchDAC();
  return true;
}

bool RaspberryPi::initialize() {

  return true;
}

bool RaspberryPi::close() {
  return true;
}





