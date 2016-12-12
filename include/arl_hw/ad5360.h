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


class AD5360 {
public:

  AD5360(Embedded_SPI *dev);

  void writeCommand(int cs);

  void setVoltage(int cs, uint8_t group, uint8_t channel, double voltage);

  void setNormalized(int cs, uint8_t group, uint8_t channel, double value);

  void buildDataCommandHeader(uint8_t group, uint8_t channel);

  void buildDataCommandValue(double voltage);

private:

  Embedded_SPI *_dev;

  uint8_t _spi_tx_buffer[AD5360_SPI_TX_BUFFER_LEN];

};

#endif
