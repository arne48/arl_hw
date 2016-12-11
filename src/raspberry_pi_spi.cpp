#include <arl_hw/raspberry_pi_spi.h>
#include <bcm2835.h>
#include <ros/ros.h>

RaspberryPi_SPI::RaspberryPi_SPI(){

    if (!bcm2835_spi_begin())
    {
      ROS_ERROR("bcm2835_spi_begin failed. Are you running as root??");
    }
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);     
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                 
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8); 
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);                
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS_NONE, LOW);

    bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(RPI_GPIO_P1_24, HIGH);



    //1100 0000 1100 0000 0000 0000
    

    //char data[3] = {0xC0,0x00,0x00};

};

RaspberryPi_SPI::~RaspberryPi_SPI(){
    bcm2835_spi_end();
};


bool RaspberryPi_SPI::setGPIO(int port, bool state){
  return true;
}

bool RaspberryPi_SPI::transferSPI(int data_len, char data[]){
    bcm2835_gpio_write(RPI_GPIO_P1_24, LOW);
    bcm2835_spi_transfern(data, data_len);
    bcm2835_gpio_write(RPI_GPIO_P1_24, HIGH);
 return true;
}
