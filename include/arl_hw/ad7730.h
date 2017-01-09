#ifndef ARL_HW_AD7730_H
#define ARL_HW_AD7730_H

#include <math.h>
#include <stdint.h>
#include <arl_hw/embedded_spi.h>

#define AD7730_SPI_TX_BUFFER_LEN 5
#define AD7730_SPI_RX_BUFFER_LEN 32


//Flags
#define AD7730_R_FLAG 0b10000000
#define AD7730_W_FLAG 0b00000000

//Commands
#define AD7730_SET_CHANNELS_COMMAND 0b00000000
#define AD7730_CONFIG_ALL_TRANSDUCERS_COMMAND 0b00000001

//Configurations

class AD7730 {
public:

  struct transducer_config{
    bool sth;
    bool sth_else;
    uint8_t blub;
  };

  AD7730(Embedded_SPI *dev);

  void setActiveChannelsByMask(int cs, uint16_t mask);

  void setConfigurationToAllTransducers(int cs, transducer_config config);

  void readData(int cs, uint8_t* data);

  void startMeasurements(int cs);

private:
  void clearBuffer();

  void writeCommand(int cs);
  Embedded_SPI *dev_;
  uint8_t spi_tx_buffer_[AD7730_SPI_TX_BUFFER_LEN];
  char spi_rx_buffer_[AD7730_SPI_RX_BUFFER_LEN];


};


#endif //AD7730_H
