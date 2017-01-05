#ifndef ARL_HW_AD7616_H
#define ARL_HW_AD7616_H

#include <math.h>
#include <stdint.h>
#include <arl_hw/embedded_spi.h>

class AD7616 {
public:
  /**
   * Constructor
   * @param dev interface to use the platform's SPI bus
   */
  AD7616(Embedded_SPI *dev);

  uint32_t getMeasurementPair(int cs, uint8_t channel);

private:
  Embedded_SPI *_dev;
};


#endif //ARL_HW_AD7616_H
