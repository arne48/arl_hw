#ifndef ARL_HW_RASPBERRYPI_SPI_H
#define ARL_HW_RASPBERRYPI_SPI_H

#include <arl_hw/embedded_spi.h>

/**
 * Implementation of the embedded SPI interface
 */
class RaspberryPi_SPI : public Embedded_SPI {
public:
  /**
   * Default Constructor
   */
  RaspberryPi_SPI();

  /**
   * Destructor
   */
  ~RaspberryPi_SPI();

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
  
};


#endif //ARL_HW_RASPBERRYPI_SPI_H
