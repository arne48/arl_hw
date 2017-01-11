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

#define AD5360_DEFAULT_GAIN 0xFFFF
#define AD5360_DEFAULT_OFFSET 0x8000

/**
 * Represents the AD5360 Digital Analog Converter of Analog Devices.
 * Provides functionalities so set output voltage based on a 3V reference voltage
 * and a convenience function to set a normalized activation between -1 and 1 over
 * the positive voltage span [e.g. 0-10V].
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

  /**
   * Sets the trim gain of the specified channel
   * @param cs chip-select id of DAC
   * @param group group on which the controlled output is localized
   * @param channel channel in group of controlled output
   * @param gain
   */
  void setGain(int cs, uint8_t group, uint8_t channel, uint16_t gain);

  /**
   * Sets the offset which is applied to the dac value - voltage mapping
   * @param cs chip-select id of DAC
   * @param group group on which the controlled output is localized
   * @param channel channel in group of controlled output
   * @param offset
   */
  void setOffset(int cs, uint8_t group, uint8_t channel, uint16_t offset);

  /**
   * Resets the offset and gain of a channel back to it's default values
   * @param cs chip-select id of DAC
   * @param group group on which the controlled output is localized
   * @param channel channel in group of controlled output
   */
  void reset(int cs, uint8_t group, uint8_t channel);

private:
  Embedded_SPI *_dev;
  uint8_t _spi_tx_buffer[AD5360_SPI_TX_BUFFER_LEN];

  void buildDataCommandHeader(uint8_t group, uint8_t channel);

  void buildGainCommandHeader(uint8_t group, uint8_t channel);

  void buildOffsetCommandHeader(uint8_t group, uint8_t channel);

  void buildDataCommandValue(double voltage);

  void buildCalibrationCommandValue(uint16_t value);

  void writeCommand(int cs);

  double map(double x, double in_min, double in_max, double out_min, double out_max);

};

#endif
