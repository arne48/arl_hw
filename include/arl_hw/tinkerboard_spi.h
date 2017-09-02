#ifndef ARL_HW_TINKERBOARD_SPI_H
#define ARL_HW_TINKERBOARD_SPI_H

#include <arl_hw/embedded_spi.h>
#include <arl_hw/tinkerboard_gpio.h>
#include <tinkerboard_io.h>

/**
 * Implementation of the embedded SPI interface
 */
class TinkerBoard_SPI : public Embedded_SPI {
public:
  TinkerBoard_SPI(TinkerBoard_GPIO *gpio);

  /**
   * Destructor
   */
  ~TinkerBoard_SPI();

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

private:
  TinkerBoard_GPIO *_gpio;
  struct spi_mode_config_t _mode;
};


#endif //ARL_HW_TINKERBOARD_SPI_H
