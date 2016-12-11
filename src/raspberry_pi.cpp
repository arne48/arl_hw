#include <arl_hw/raspberry_pi.h>
#include <ros/ros.h>
#include <bcm2835.h>

RaspberryPi::RaspberryPi() {

    if (!bcm2835_init()) {
      ROS_ERROR("bcm2835_init failed. Are you running as root??");
    }

  _spi = new RaspberryPi_SPI();
  //_adc_vec.push_back(new AD5360(0,5,_spi));

  //_adc_vec[0]->setVoltage(BANKALL,CHALL,-3.0);

  char data[3] = {0xC0,0xC0,0x00};
  _spi->transferSPI(3, data);
}

RaspberryPi::~RaspberryPi() {
  delete _spi;
  bcm2835_close();
}

bool RaspberryPi::read(std::vector<arl_datatypes::muscle_status_data_t> &status) {
  //_adc_vec[0]->setVoltage(BANKALL,CHALL,3.0);
  //_adc_vec[0]->latchDAC();
  return true;
}

bool RaspberryPi::write(std::vector<arl_datatypes::muscle_command_data_t> &command) {
  //_adc_vec[0]->setVoltage(BANKALL,CHALL,1.0);
  //_adc_vec[0]->latchDAC();
  return true;
}

bool RaspberryPi::initialize() {

  return true;
}

bool RaspberryPi::close() {
  return true;
}





