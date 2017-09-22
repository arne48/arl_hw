#ifndef ARL_HW_AD7616_H
#define ARL_HW_AD7616_H

#include <math.h>
#include <stdint.h>
#include <arl_hw/embedded_spi.h>
#include <arl_hw/embedded_gpio.h>

class AD7616 {
public:
  /**
   * Constructor
   * @param dev interface to use the platform's SPI bus
   */
  AD7616(Embedded_SPI *dev, Embedded_GPIO *gpio, int trigger_measurement_gpio);

  /**
   * Issues a measurement of a certain channel pair [VAx & VBx]
   * @param cs cs chip-select id of ADC
   * @param channel_pair the channel-pair to get a measurement from
   * @return
   */
  uint32_t getMeasurementPair(int cs, uint8_t channel_pair);

private:
  Embedded_SPI *_dev;
  Embedded_GPIO *_gpio;
  int _trigger_measurement_gpio;

  void prepareChannel(uint8_t channel, int cs);

  char _rx_buffer[4] = {0,0,0,0};
  char _tx_buffer[4] = {0,0,0,0};
  char _channel_select_command[4] = {(char)0x86, (char)0x00, (char)0x86, (char)0x00};


};


#endif //ARL_HW_AD7616_H
