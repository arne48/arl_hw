#include <arl_hw/ad7616.h>

AD7616::AD7616(Embedded_SPI *dev) {
  _dev = dev;
}

void AD7616::prepareChannel(uint8_t channel, int cs){
  _channel_select_command[1] = channel | channel << 4;
  _channel_select_command[3] = channel | channel << 4;

  _dev->transferSPI(cs, 4, _channel_select_command, _rx_buffer);

  //Dummy read to get rid of values of the last measurement
  bcm2835_gpio_write(RPI_GPIO_P1_16, LOW);
  bcm2835_gpio_write(RPI_GPIO_P1_16, HIGH);


  _dev->transferSPI(cs, 4, _channel_select_command, _rx_buffer);
}

uint32_t  AD7616::getMeasurementPair(int cs, uint8_t channel){
  prepareChannel(channel, cs);

  bcm2835_gpio_write(RPI_GPIO_P1_16, LOW);
  bcm2835_gpio_write(RPI_GPIO_P1_16, HIGH);

  _dev->transferSPI(cs, 4, _tx_buffer, _rx_buffer);

  uint32_t  ret = 0;
  ret = (uint8_t)_rx_buffer[0] << 24 | (uint8_t)_rx_buffer[1] << 16 | (uint8_t)_rx_buffer[2] << 8 | (uint8_t)_rx_buffer[3];

  return ret;
}

void AD7616::getAllMeasurements(int cs, uint16_t data[]){
  bcm2835_gpio_write(RPI_GPIO_P1_16, LOW);
  bcm2835_gpio_write(RPI_GPIO_P1_16, HIGH);
  delay(1);

  for(int i = 0; i<8; i++){
    bcm2835_gpio_write(cs, LOW);
    bcm2835_spi_transfernb(_tx_buffer, _rx_buffer, 4);
    bcm2835_gpio_write(cs, HIGH);
    data[i] = (uint8_t)_rx_buffer[0] << 8 | (uint8_t)_rx_buffer[1];
    data[i+8] = (uint8_t)_rx_buffer[1] << 8 | (uint8_t)_rx_buffer[2];
  }
}

void AD7616::setupSequencer(int cs){
  bcm2835_gpio_write(cs, LOW);
  bcm2835_spi_transfernb(_seq_burst_command, _rx_buffer, 2);
  bcm2835_gpio_write(cs, HIGH);

  bcm2835_gpio_write(cs, LOW);
  bcm2835_spi_transfernb(_command_read, _rx_buffer, 2);
  bcm2835_gpio_write(cs, HIGH);
  bcm2835_gpio_write(cs, LOW);
  bcm2835_spi_transfernb(_tx_buffer, _rx_buffer, 2);
  bcm2835_gpio_write(cs, HIGH);

  delay(100);
}