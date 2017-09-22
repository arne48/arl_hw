#include <arl_hw/ad7616.h>

AD7616::AD7616(Embedded_SPI *dev, Embedded_GPIO *gpio, int trigger_measurement_gpio) {
  _dev = dev;
  _gpio = gpio;
  _trigger_measurement_gpio = trigger_measurement_gpio;
}

void AD7616::prepareChannel(uint8_t channel, int cs){
  _channel_select_command[1] = channel | channel << 4;
  _channel_select_command[3] = channel | channel << 4;

  _dev->transferSPI(cs, 4, _channel_select_command, _rx_buffer);

  //Dummy read to get rid of values of the last measurement
  _gpio->set_output(_trigger_measurement_gpio, Embedded_GPIO::gpio_state::OFF);
  _gpio->set_output(_trigger_measurement_gpio, Embedded_GPIO::gpio_state::ON);


  _dev->transferSPI(cs, 4, _channel_select_command, _rx_buffer);
}

uint32_t  AD7616::getMeasurementPair(int cs, uint8_t channel){
  prepareChannel(channel, cs);

  _gpio->set_output(_trigger_measurement_gpio, Embedded_GPIO::gpio_state::OFF);
  _gpio->set_output(_trigger_measurement_gpio, Embedded_GPIO::gpio_state::ON);

  _dev->transferSPI(cs, 4, _tx_buffer, _rx_buffer);

  uint32_t  ret = 0;
  ret = (uint8_t)_rx_buffer[0] << 24 | (uint8_t)_rx_buffer[1] << 16 | (uint8_t)_rx_buffer[2] << 8 | (uint8_t)_rx_buffer[3];

  return ret;
}