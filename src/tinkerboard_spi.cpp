#include <arl_hw/tinkerboard_spi.h>
#include <ros/ros.h>

TinkerBoard_SPI::TinkerBoard_SPI(TinkerBoard_GPIO *gpio) : Embedded_SPI(gpio)  {
  _gpio = gpio;


  struct spi_mode_config_t mode;
  mode.clk_mode = 1;
  mode.clk_divider = 8;
  mode.data_frame_size = SPIDataFrameSize::DFS_8;
  mode.slave_select = SPISlaveSelect::SS_NONE;
  mode.transfer_mode = SPITransferMode::TRANSMIT_RECEIVE;
  mode.byte_order = SPIByteOrder::MSB_FIRST;

  _mode = mode;

  tinkerboard_spi_init(SPI2, mode);

};

TinkerBoard_SPI::~TinkerBoard_SPI() {
  tinkerboard_spi_end(SPI2);
};

bool TinkerBoard_SPI::transferSPI(int cs, int data_len, char data[]) {
  uint8_t data_rx[data_len];
  uint8_t data_tx[data_len];
  for(unsigned int i = 0; i < data_len; i++) {
    data_tx[i] = data[i];
  }

  tinkerboard_set_gpio_state((uint32_t) cs, IOState::LOW);
  tinkerboard_spi_transfer(SPI2, data_tx, data_rx, data_len, _mode);
  tinkerboard_set_gpio_state((uint32_t)cs, IOState::HIGH);
  return true;
}

bool TinkerBoard_SPI::transferSPI(int cs, int data_len, char data_tx[], char data_rx[]) {
  uint8_t data_rx_tmp[data_len];
  uint8_t data_tx_tmp[data_len];
  for(unsigned int i = 0; i < data_len; i++) {
    data_tx_tmp[i] = data_tx[i];
  }

  tinkerboard_set_gpio_state((uint32_t) cs, IOState::LOW);
  tinkerboard_spi_transfer(SPI2, data_tx_tmp, data_rx_tmp, data_len, _mode);
  tinkerboard_set_gpio_state((uint32_t) cs, IOState::HIGH);

  for(unsigned int i = 0; i < data_len; i++) {
    data_rx[i] = data_rx_tmp[i];
  }
  return true;
}
