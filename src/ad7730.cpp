#include "arl_hw/ad7730.h"

AD7730::AD7730(Embedded_SPI *dev) {
  dev_ = dev;
}

void AD7730::setActiveChannelsByMask(int cs, uint16_t mask){
  clearBuffer();
  spi_tx_buffer_[0] = AD7730_W_FLAG;
  spi_tx_buffer_[0] = spi_tx_buffer_[0] | (uint8_t) AD7730_SET_CHANNELS_COMMAND;
  spi_tx_buffer_[1] = mask;
  writeCommand(cs);
}

void AD7730::setConfigurationToAllTransducers(int cs, transducer_config config){
  clearBuffer();
  spi_tx_buffer_[0] = AD7730_W_FLAG;
  spi_tx_buffer_[0] = spi_tx_buffer_[0] | (uint8_t) AD7730_CONFIG_ALL_TRANSDUCERS_COMMAND;

  /*
   * TODO: add setting of config-parameters
   */

  writeCommand(cs);
}

void AD7730::startMeasurements(int cs){
  clearBuffer();
  spi_tx_buffer_[0] = AD7730_R_FLAG;
  writeCommand(cs);
}

void AD7730::readData(int cs, uint8_t* data){
  dev_->transferSPI(cs, 2, spi_rx_buffer_);

  int size = spi_rx_buffer_[0];
  dev_->transferSPI(cs, size * 2, spi_rx_buffer_);
  for(int i=0; i < size * 2; i++) {
    data[i] = spi_rx_buffer_[i];
  }
}

void AD7730::writeCommand(int cs){
  char data[AD7730_SPI_TX_BUFFER_LEN];
  data[0] = spi_tx_buffer_[0];
  data[1] = spi_tx_buffer_[1];
  data[2] = spi_tx_buffer_[2];
  data[3] = spi_tx_buffer_[3];
  dev_->transferSPI(cs, AD7730_SPI_TX_BUFFER_LEN, data);
}

void AD7730::clearBuffer(){
  for(int i=0; i < AD7730_SPI_TX_BUFFER_LEN; i++){
    spi_tx_buffer_[i] = 0;
  }
}

