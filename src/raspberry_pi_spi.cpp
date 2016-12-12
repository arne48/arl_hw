#include <arl_hw/raspberry_pi_spi.h>
#include <bcm2835.h>
#include <ros/ros.h>

RaspberryPi_SPI::RaspberryPi_SPI() {

  if (!bcm2835_spi_begin()) {
    ROS_ERROR("bcm2835_spi_begin failed. Are you running as root??");
  }
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS_NONE, LOW);

  //1100 0000 1100 0000 0000 0000


  //char data[3] = {0xC0,0x00,0x00};

};

RaspberryPi_SPI::~RaspberryPi_SPI() {
  bcm2835_spi_end();
};

bool RaspberryPi_SPI::transferSPI(int cs, int data_len, char data[]) {
  bcm2835_gpio_write(cs, LOW);
  bcm2835_spi_transfern(data, data_len);
  bcm2835_gpio_write(cs, HIGH);
  return true;
}
