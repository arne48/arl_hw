#include <arl_hw/raspberry_pi_spi.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

RaspberryPi_SPI::RaspberryPi_SPI(){
  wiringPiSPISetup(0, 300000);

};

RaspberryPi_SPI::~RaspberryPi_SPI(){};


bool RaspberryPi_SPI::setGPIO(int port, bool state){
  if(state){
    digitalWrite(port, EMBD_SPI_HIGH);
  } else{
    digitalWrite(port, EMBD_SPI_LOW);
  }
  return true;
}

bool RaspberryPi_SPI::transferSPI(int data_len, unsigned char data[]){
  wiringPiSPIDataRW(1, data, data_len);
}
