#ifndef ARL_HW_AD7616_H
#define ARL_HW_AD7616_H

#include <math.h>
#include <stdint.h>
#include <arl_hw/embedded_spi.h>
#include <bcm2835.h>

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

  void getAllMeasurements(int cs, uint16_t data[]);
  void setupSequencer(int cs);

private:
  Embedded_SPI *_dev;
  void prepareChannel(uint8_t channel, int cs);

  char _rx_buffer[4] = {0,0,0,0};
  char _tx_buffer[4] = {0,0,0,0};
  char _channel_select_command[4] = {(char)0x86, (char)0x00, (char)0x86, (char)0x00};
  char _seq_burst_command[2] = {(char)0x84, (char)0x60};
  char _command_read[2] = {(char)0x04,(char) 0x00};


};


#endif //ARL_HW_AD7616_H
