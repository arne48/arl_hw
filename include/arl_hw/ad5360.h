#ifndef AD5360_h
#define AD5360_h

#include <math.h>
#include <stdint.h>
#include <arl_hw/embedded_spi.h>

#define AD5360_SPI_TX_BUFFER_LEN 3

#define AD5360_REF_VOLTAGE 3.0

#define AD5360_MODE_WRITE_DAC_DATA 0b11000000
#define AD5360_MODE_WRITE_DAC_OFFSET 0b10000000
#define AD5360_MODE_WRITE_DAC_GAIN 0b01000000
#define AD5360_MODE_WRITE_SPECIAL_FUNCTION 0b00000000

/**
 * Represents the AD5360 Digital Analog Converter of Analog Devices.
 * Provides functionalities so set output voltage based on a 3V reference voltage
 * and a convenience function to set a normalized activation between -1 and 1 over
 * the full voltage span.
 */
class AD5360 {
public:

  /**
   * Constructor
   * @param dev interface to use the platform's SPI bus
   */
  AD5360(Embedded_SPI *dev);

  /**
   * Sets voltage for a specific channel (0-7) on group A(0) or B(1)
   * of a DAC using a given chip-select id (starting at 0)
   * @param cs chip-select id of DAC
   * @param group group on which the controlled output is localized
   * @param channel channel in group of controlled output
   * @param voltage voltage to set (at 3V Vref -> -6V to 6V)
   */
  void setVoltage(int cs, uint8_t group, uint8_t channel, double voltage);

  /**
   * Sets a normalized activation for a specific channel (0-7) on group A(0) or B(1)
   * of a DAC using a given chip-select id (starting at 0)
   * @param cs chip-select id of DAC
   * @param group group on which the controlled output is localized
   * @param channel channel in group of controlled output
   * @param value value of linear mapping to set output (-1 to 1)
   */
  void setNormalized(int cs, uint8_t group, uint8_t channel, double value);

private:

  void buildDataCommandHeader(uint8_t group, uint8_t channel);

  void buildDataCommandValue(double voltage);

  void writeCommand(int cs);

  Embedded_SPI *_dev;

  uint8_t _spi_tx_buffer[AD5360_SPI_TX_BUFFER_LEN];

};

#endif
