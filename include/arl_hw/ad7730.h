#ifndef ARL_HW_AD7730_H
#define ARL_HW_AD7730_H

#include <math.h>
#include <stdint.h>
#include <arl_hw/embedded_spi.h>

#define AD7730_SPI_TX_BUFFER_LEN 5
#define AD7730_SPI_RX_BUFFER_LEN 64


//Flags
#define AD7730_R_FLAG 0b10000000
#define AD7730_W_FLAG 0b00000000

//Commands
#define AD7730_SET_CHANNELS_COMMAND 0b00000000
#define AD7730_CONFIG_ALL_TRANSDUCERS_COMMAND 0b00000001

//Configurations

class AD7730 {
public:

  /**
   * Constructor
   * @param dev interface to use the platform's SPI bus
   */
  AD7730(Embedded_SPI *dev);

  /**
   * Reads measurement data of all activated channels on specified load cell board
   * @param cs chip-select id of load cell board
   * @param data data-buffer
   */
  void readData(int cs, uint8_t* data);


private:
  Embedded_SPI *dev_;
  uint8_t spi_tx_buffer_[AD7730_SPI_TX_BUFFER_LEN];
  char spi_rx_buffer_[AD7730_SPI_RX_BUFFER_LEN];

};


#endif //ARL_HW_AD7730_H
