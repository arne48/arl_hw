#include "arl_hw/ad7730.h"

AD7730::AD7730(Embedded_SPI *dev) {
  dev_ = dev;
  tx_command_buffer_[0] = (char) 0xFD;
}

void AD7730::readData(int cs, uint8_t *data) {
  dev_->transferSPI(cs, AD7730_SPI_RX_BUFFER_LEN, tx_command_buffer_, spi_rx_buffer_);

  for (uint8_t idx = 0; idx < AD7730_SPI_RX_BUFFER_LEN; idx++) {
    data[idx] = (uint8_t) spi_rx_buffer_[idx];
  }
}