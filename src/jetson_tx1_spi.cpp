#include <arl_hw/jetson_tx1_spi.h>
#include <ros/ros.h>

JetsonTX1_SPI::JetsonTX1_SPI() {
  _enable = 0;
  _latch = 0;
  _address_length = 0;

  _spi_config = {
    device : "/dev/spidev0.0",
    mode : 0,
    bits : 8,
    speed : 32000000, /*32MHz*/
    delay : 0
  };

  _spi_descriptor = open(_spi_config.device, O_RDWR);
  int ret = ioctl(_spi_descriptor, SPI_IOC_WR_MODE, _spi_config.mode);
  ret = ioctl(_spi_descriptor, SPI_IOC_RD_MODE, _spi_config.mode);
  ret = ioctl(_spi_descriptor, SPI_IOC_WR_BITS_PER_WORD, _spi_config.bits);
  ret = ioctl(_spi_descriptor, SPI_IOC_RD_BITS_PER_WORD, _spi_config.bits);
  ret = ioctl(_spi_descriptor, SPI_IOC_WR_MAX_SPEED_HZ, _spi_config.speed);
  ret = ioctl(_spi_descriptor, SPI_IOC_RD_MAX_SPEED_HZ, _spi_config.speed);


}

JetsonTX1_SPI::JetsonTX1_SPI(JetsonTX1_GPIO *gpio) {
  _gpio = gpio;
  close(_spi_descriptor);
}

JetsonTX1_SPI::~JetsonTX1_SPI() {
}

bool JetsonTX1_SPI::transferSPI(int cs, int data_len, char data[]) {

  //setting chip-select
  _gpio->setCSByMultiplexerAddress(cs, _multiplexer_address_bus, _address_length, _enable, _latch);

  //write
  char rx[data_len] = {};

  struct spi_ioc_transfer_t _spi_ioc_transfer= {
    tx_buf : (unsigned long) data,
    rx_buf : (unsigned long) rx,
    len : (uint32_t) data_len,
    delay_usecs : _spi_config.delay,
    speed_hz : _spi_config.speed,
    bits_per_word : _spi_config.bits
  };

  int ret = ioctl(_spi_descriptor, SPI_IOC_MESSAGE(1), &_spi_ioc_transfer);

  //resetting chip-select
  _gpio->resetCSByMultiplexer(_enable);
  return true;
}

bool JetsonTX1_SPI::transferSPI(int cs, int data_len, char data_tx[], char data_rx[]){
 return true;
}


void JetsonTX1_SPI::setSCLKDivider(int divider) {
}

void JetsonTX1_SPI::setMultiplexerBusAddresses(int *bus_addresses, int address_length, int enable, int latch){
  _multiplexer_address_bus = bus_addresses;
  _address_length = address_length;
  _enable = enable;
  _latch = latch;
}
