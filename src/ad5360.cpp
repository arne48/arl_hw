#include <arl_hw/ad5360.h>
#include <ros/ros.h>

AD5360::AD5360(Embedded_SPI *dev) {
  _dev = dev;
}


void AD5360::setVoltage(int cs, uint8_t group, uint8_t channel, double voltage) {
  ROS_DEBUG("AD5360: Using GPIO %d, Channel %d, Group %d", cs, channel, group);
  buildDataCommandHeader(group, channel);
  buildDataCommandValue(voltage);
  writeCommand(cs);
}

void AD5360::setNormalized(int cs, uint8_t group, uint8_t channel, double value) {
  ROS_DEBUG("AD5360: Using GPIO %d, Channel %d, Group %d", cs, channel, group);
  double voltage = value / (1 / (AD5360_REF_VOLTAGE * 2));
  buildDataCommandHeader(group, channel);
  buildDataCommandValue(voltage);
  writeCommand(cs);
}

void AD5360::writeCommand(int cs) {

  char data[3];
  data[0] = _spi_tx_buffer[0];
  data[1] = _spi_tx_buffer[1];
  data[2] = _spi_tx_buffer[2];
  _dev->transferSPI(cs, 3, data);

}

void AD5360::buildDataCommandHeader(uint8_t group, uint8_t channel) {
  _spi_tx_buffer[0] = AD5360_MODE_WRITE_DAC_DATA;
  _spi_tx_buffer[0] = _spi_tx_buffer[0] | (group + (uint8_t) 1) << 3;
  _spi_tx_buffer[0] = _spi_tx_buffer[0] | channel;
}

void AD5360::buildDataCommandValue(double voltage) {

  if (voltage == 0.0) {
    uint16_t command_data = pow(2, 16) / 2;
    _spi_tx_buffer[1] = (uint8_t) ((command_data & 0xFF00) >> 8);
    _spi_tx_buffer[2] = (uint8_t) (command_data & 0x00FF);
  } else if (voltage <= (2 * AD5360_REF_VOLTAGE) * -1) {
    _spi_tx_buffer[1] = 0;
    _spi_tx_buffer[2] = 0;
  } else if (voltage >= (2 * AD5360_REF_VOLTAGE)) {
    uint16_t command_data = pow(2, 16) - 1;
    _spi_tx_buffer[1] = (uint8_t) ((command_data & 0xFF00) >> 8);
    _spi_tx_buffer[2] = (uint8_t) (command_data & 0x00FF);
  } else {

    uint16_t command_data = pow(2, 16) / 2;
    uint16_t value = fabs(voltage) / ((AD5360_REF_VOLTAGE * 2) / pow(2, 15));

    if (voltage > 0) {
      command_data += value;
    } else {
      command_data -= value;
    }

    _spi_tx_buffer[1] = (uint8_t) ((command_data & 0xFF00) >> 8);
    _spi_tx_buffer[2] = (uint8_t) (command_data & 0x00FF);
  }
}
