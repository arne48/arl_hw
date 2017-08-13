#include <arl_hw/linux_platform_spi.h>
#include <ros/ros.h>

LinuxPlatform_SPI::LinuxPlatform_SPI(LinuxPlatform_GPIO *gpio) : Embedded_SPI(gpio) {
  _gpio = gpio;
  _device = std::string("/dev/spidev1.1");
  _mode = SPI_MODE_1;
  _speed = 1600000;
  _bits = 8;

  _spi_descriptor = open(_device.c_str(), O_RDWR);
  if(_spi_descriptor < 0) {
    ROS_FATAL("Couldn't open SPI device");
  }

  int ret = 0;
  ret = ioctl(_spi_descriptor, SPI_IOC_WR_MODE, &_mode);
  if(ret < 0) {
    ROS_FATAL("Couldn't set SPI write mode");
  }
  ret = ioctl(_spi_descriptor, SPI_IOC_RD_MODE, &_mode);
  if(ret < 0) {
    ROS_FATAL("Couldn't set SPI read mode");
  }

  ret = ioctl(_spi_descriptor, SPI_IOC_WR_BITS_PER_WORD, &_bits);
  if(ret < 0) {
    ROS_FATAL("Couldn't set SPI write word length");
  }
  ret = ioctl(_spi_descriptor, SPI_IOC_RD_BITS_PER_WORD, &_bits);
  if(ret < 0) {
    ROS_FATAL("Couldn't set SPI read word length");
  }

  ret = ioctl(_spi_descriptor, SPI_IOC_WR_MAX_SPEED_HZ, &_speed);
  if(ret < 0) {
    ROS_FATAL("Couldn't set SPI write speed");
  }
  ret = ioctl(_spi_descriptor, SPI_IOC_RD_MAX_SPEED_HZ, &_speed);
  if(ret < 0) {
    ROS_FATAL("Couldn't set SPI read speed");
  }

  _spi_ioc_transfer.delay_usecs = 0;
  _spi_ioc_transfer.speed_hz = 16000000;
  _spi_ioc_transfer.bits_per_word = 8;
}

LinuxPlatform_SPI::~LinuxPlatform_SPI() {
  close(_spi_descriptor);
}

bool LinuxPlatform_SPI::transferSPI(int cs, int data_len, char data[]) {
  _gpio->set_output(cs, Embedded_GPIO::gpio_state::OFF);
  char rx[data_len] = {};

  _spi_ioc_transfer.tx_buf = (unsigned long)data;
  _spi_ioc_transfer.rx_buf = (unsigned long)rx;
  _spi_ioc_transfer.len = (uint32_t)data_len;

  int ret = ioctl(_spi_descriptor, SPI_IOC_MESSAGE(1), &_spi_ioc_transfer);
  if(ret < 0) {
    ROS_FATAL("Couldn't sent data over SPI");
  }

  _gpio->set_output(cs, Embedded_GPIO::gpio_state::ON);
  return true;
}

bool LinuxPlatform_SPI::transferSPI(int cs, int data_len, char data_tx[], char data_rx[]){
  _gpio->set_output(cs, Embedded_GPIO::gpio_state::OFF);

  _spi_ioc_transfer.tx_buf = (unsigned long)data_tx;
  _spi_ioc_transfer.rx_buf = (unsigned long)data_rx;
  _spi_ioc_transfer.len = (uint32_t)data_len;

  int ret = ioctl(_spi_descriptor, SPI_IOC_MESSAGE(1), &_spi_ioc_transfer);
  if(ret < 0) {
    ROS_FATAL("Couldn't send and receive data over SPI");
  }

  _gpio->set_output(cs, Embedded_GPIO::gpio_state::ON);
 return true;
}
