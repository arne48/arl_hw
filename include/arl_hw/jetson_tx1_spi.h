#ifndef ARL_HW_JETSON_TX1_SPI_H
#define ARL_HW_JETSON_TX1_SPI_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <arl_hw/embedded_spi.h>
#include <arl_hw/jetson_tx1_gpio.h>

/**
 * Implementation of the embedded SPI interface
 */
class JetsonTX1_SPI : public Embedded_SPI  {
public:

  struct spi_config_t {
    const char *device;
    uint8_t mode;
    uint8_t bits;
    uint32_t speed;
    uint16_t delay;
  };

  struct spi_ioc_transfer_t {
    unsigned long tx_buf;
    unsigned long rx_buf;
    uint32_t len;
    uint16_t delay_usecs;
    uint32_t speed_hz;
    uint8_t bits_per_word;
  };

  /**
   * Default Constructor
   */
  JetsonTX1_SPI();

  JetsonTX1_SPI(JetsonTX1_GPIO *gpio);

  /**
   * Destructor
   */
  ~JetsonTX1_SPI();

  /**
   * Call to transfer an array of bytes to a given chip-select "id"
   * The mapping of those "ids" starting by 0 has to be performed by
   * the implementing class.
   * @param cs chip-select
   * @param data_len length of data array
   * @param data array containing the data to transfer
   * @return true if transfer was executed successfully
   */
  virtual bool transferSPI(int cs, int data_len, char data[]);

  /**
   * Call to transfer an array of bytes to a given native chip-select
   * using two buffers for transfer and receiving
   * @param cs chip-select
   * @param data_len length of data array
   * @param data_tx array containing the data to transfer
   * @param data_rx array containing the data to receive
   * @return true if transfer was executed successfully
   */
  virtual bool transferSPI(int cs, int data_len, char data_tx[], char data_rx[]);

  /**
   * Sets the clock prescaler of SPI device
   * @param divider value to set the frequency divider to
   */
  virtual void setSCLKDivider(int divider);

  /**
   * Sets the addresses just to access the multiplexer plus it's latch and enable signal
   * @param bus_addresses a pointer to an array of gpio numbers representing the address LSB first
   * @param address_length length of the address
   * @param enable gpio number connected to the multiplexer's enable signal
   * @param latch gpio number connected to the multiplexer's latch signal
   */
  void setMultiplexerBusAddresses(int *bus_addresses, int address_length, int enable, int latch);

private:
  JetsonTX1_GPIO *_gpio;

  int *_multiplexer_address_bus;
  int _address_length;
  int _enable;
  int _latch;

  int _spi_descriptor;
  spi_config_t _spi_config;
  spi_ioc_transfer _spi_ioc_transfer;
};


#endif //ARL_HW_JETSON_TX1_SPI_H
