#include <arl_hw/raspberry_pi.h>
#include <ros/ros.h>
#include <bcm2835.h>

RaspberryPi::RaspberryPi() {

  if (!bcm2835_init()) {
    ROS_ERROR("bcm2835_init failed. Are you running as root??");
  }

  _spi = new RaspberryPi_SPI();


  //Chip-Selects
  bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_26, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_32, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_36, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_07, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_11, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_13, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_15, BCM2835_GPIO_FSEL_OUTP);


  bcm2835_gpio_write(RPI_GPIO_P1_24, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_26, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_32, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_36, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_07, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_11, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_13, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_15, HIGH);

  //Latch
  bcm2835_gpio_fsel(RPI_GPIO_P1_18, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(RPI_GPIO_P1_18, LOW);



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
  char data[3] = {(char)0xC0, (char)0xC0, (char)0x00};
  _spi->transferSPI(RPI_GPIO_P1_24, 3, data);
  return true;
}

bool RaspberryPi::initialize() {

  return true;
}

bool RaspberryPi::close() {
  return true;
}





