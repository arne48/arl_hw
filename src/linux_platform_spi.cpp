#include <arl_hw/linux_platform_spi.h>
#include <ros/ros.h>

LinuxPlatform_SPI::LinuxPlatform_SPI(LinuxPlatform_GPIO *gpio) : Embedded_SPI(gpio) {
  _gpio = gpio;

  _spi_config = {
    device : "/dev/spidev1.1",
    mode : 1,
    bits : 8,
    speed : 15000000, /*15MHz*/
    delay : 0
  };

/*  _spi_descriptor = open(_spi_config.device, O_RDWR);
  int ret = ioctl(_spi_descriptor, SPI_IOC_WR_MODE, _spi_config.mode);
  ret = ioctl(_spi_descriptor, SPI_IOC_RD_MODE, _spi_config.mode);
  ret = ioctl(_spi_descriptor, SPI_IOC_WR_BITS_PER_WORD, _spi_config.bits);
  ret = ioctl(_spi_descriptor, SPI_IOC_RD_BITS_PER_WORD, _spi_config.bits);
  ret = ioctl(_spi_descriptor, SPI_IOC_WR_MAX_SPEED_HZ, _spi_config.speed);
  ret = ioctl(_spi_descriptor, SPI_IOC_RD_MAX_SPEED_HZ, _spi_config.speed);*/
}

LinuxPlatform_SPI::~LinuxPlatform_SPI() {
  //close(_spi_descriptor);
}

bool LinuxPlatform_SPI::transferSPI(int cs, int data_len, char data[]) {
  //_gpio->set_output(cs, Embedded_GPIO::gpio_state::OFF);
/*  char rx[data_len] = {};

  struct spi_ioc_transfer_t _spi_ioc_transfer= {
          tx_buf : (unsigned long) data,
          rx_buf : (unsigned long) rx,
          len : (uint32_t) data_len,
          delay_usecs : _spi_config.delay,
          speed_hz : _spi_config.speed,
          bits_per_word : _spi_config.bits
  };*/

  //ioctl(_spi_descriptor, SPI_IOC_MESSAGE(1), &_spi_ioc_transfer);

  //_gpio->set_output(cs, Embedded_GPIO::gpio_state::ON);
  return true;
}

bool LinuxPlatform_SPI::transferSPI(int cs, int data_len, char data_tx[], char data_rx[]){
  //_gpio->set_output(cs, Embedded_GPIO::gpio_state::OFF);

/*  struct spi_ioc_transfer_t _spi_ioc_transfer= {
          tx_buf : (unsigned long) data_tx,
          rx_buf : (unsigned long) data_rx,
          len : (uint32_t) data_len,
          delay_usecs : _spi_config.delay,
          speed_hz : _spi_config.speed,
          bits_per_word : _spi_config.bits
  };*/

  //ioctl(_spi_descriptor, SPI_IOC_MESSAGE(1), &_spi_ioc_transfer);

  //_gpio->set_output(cs, Embedded_GPIO::gpio_state::ON);
 return true;
}
