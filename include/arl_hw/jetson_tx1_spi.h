#ifndef ARL_HW_JETSON_TX1_SPI_H
#define ARL_HW_JETSON_TX1_SPI_H

#include <arl_hw/embedded_spi.h>
#include <arl_hw/jetson_tx1_gpio.h>

/**
 * Implementation of the embedded SPI interface
 */
class JetsonTX1_SPI : public Embedded_SPI  {
public:
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
   * Sets the clock prescaler of SPI device
   * @param divider
   */
  virtual void setSCLKDivider(int divider);

private:
  JetsonTX1_GPIO *_gpio;


};


#endif //ARL_HW_JETSON_TX1_SPI_H