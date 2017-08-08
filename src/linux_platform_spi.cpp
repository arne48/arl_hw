#include <arl_hw/linux_platform_spi.h>
#include <ros/ros.h>

LinuxPlatform_SPI::LinuxPlatform_SPI() {

}

LinuxPlatform_SPI::LinuxPlatform_SPI(LinuxPlatform_GPIO *gpio) {
}

LinuxPlatform_SPI::~LinuxPlatform_SPI() {
}

bool LinuxPlatform_SPI::transferSPI(int cs, int data_len, char data[]) {
  return true;
}

bool LinuxPlatform_SPI::transferSPI(int cs, int data_len, char data_tx[], char data_rx[]){
 return true;
}

void LinuxPlatform_SPI::setSCLKDivider(int divider) {
}