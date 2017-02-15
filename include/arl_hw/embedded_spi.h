#ifndef ARL_HW_EMBEDDED_SPI_H
#define ARL_HW_EMBEDDED_SPI_H

/**
 * Interface class for accessing the SPI bus of an arbitrary platform
 */
class Embedded_SPI {
public:
  /**
   * Default Constructor
   */
  Embedded_SPI() {};

  /**
   * Destructor
   */
  virtual ~Embedded_SPI() {};

  /**
   * Call to transfer an array of bytes to a given native chip-select
   * @param cs chip-select
   * @param data_len length of data array
   * @param data array containing the data to transfer
   * @return true if transfer was executed successfully
   */
  virtual bool transferSPI(int cs, int data_len, char data[]) = 0;

  /**
   * Call to transfer an array of bytes to a given native chip-select
   * using two buffers for transfer and receiving
   * @param cs chip-select
   * @param data_len length of data array
   * @param data_tx array containing the data to transfer
   * @param data_rx array containing the data to receive
   * @return true if transfer was executed successfully
   */
  virtual bool transferSPI(int cs, int data_len, char data_tx[], char data_rx[]) = 0;

  /**
   * Sets the clock prescaler of SPI device
   * @param divider
   */
  virtual void setSCLKDivider(int divider) = 0;

};


#endif //ARL_HW_EMBEDDED_SPI_H
