#ifndef TINKERBOARD_SPI_H
#define TINKERBOARD_SPI_H

#include <arl_hw/embedded_spi.h>

/**
 * Implementation of the embedded SPI interface
 */
class TinkerBoard_SPI : public Embedded_SPI {
public:
  /**
   * Default Constructor
   */
  TinkerBoard_SPI();

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

  /**
   * Sets the clock prescaler of SPI device
   * @param divider value to set the frequency divider to
   */
  virtual void setSCLKDivider(int divider);

};


#endif //TINKERBOARD_SPI_H
