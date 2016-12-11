#ifndef ARL_HW_RASPBERRYPI_SPI_H
#define ARL_HW_RASPBERRYPI_SPI_H

#include <arl_hw/embedded_spi.h>

class RaspberryPi_SPI: public Embedded_SPI {

public:
  RaspberryPi_SPI();

  ~RaspberryPi_SPI();

  virtual bool setGPIO(int port, bool state);

  virtual bool transferSPI(int data_len, char data[]);

};


#endif //ARL_HW_RASPBERRYPI_SPI_H
