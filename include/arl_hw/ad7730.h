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
  /**
   * Datastructure to store configuration of transducers on load cell board
   */
  struct transducer_config{
    bool sth;
    bool sth_else;
    uint8_t blub;
  };

  /**
   * Constructor
   * @param dev interface to use the platform's SPI bus
   */
  AD7730(Embedded_SPI *dev);

  /**
   * Activates channel on load cell board only measurements of activated channels will be read
   * using a bitmask
   * @param cs chip-select id of load cell board
   * @param mask bitmask to apply
   */
  void setActiveChannelsByMask(int cs, uint16_t mask);

  /**
   * Sets a configuration to all transducers on one load cell board
   * @param cs chip-select id of load cell board
   * @param config configuration to set
   */
  void setConfigurationToAllTransducers(int cs, transducer_config config);

  /**
   * Reads measurement data of all activated channels on specified load cell board
   * @param cs chip-select id of load cell board
   * @param data data-buffer
   */
  void readData(int cs, uint8_t* data);

  /**
   * After configuration of load cell board this commands start the actual measurements
   * issued by the controller
   * @param cs chip-select id of load cell board
   */
  void startMeasurements(int cs);

private:
  Embedded_SPI *dev_;
  uint8_t spi_tx_buffer_[AD7730_SPI_TX_BUFFER_LEN];
  char spi_rx_buffer_[AD7730_SPI_RX_BUFFER_LEN];

  void clearBuffer();

  void writeCommand(int cs);

};


#endif //ARL_HW_AD7730_H
