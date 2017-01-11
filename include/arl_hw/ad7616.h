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

  /**
   * Issues a measurement of a certain channel pair [VAx & VBx]
   * @param cs cs chip-select id of ADC
   * @param channel_pair the channel-pair to get a measurement from
   * @return
   */
  uint32_t getMeasurementPair(int cs, uint8_t channel_pair);

private:
  Embedded_SPI *_dev;

};


#endif //ARL_HW_AD7616_H
