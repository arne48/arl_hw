#include <arl_hw/ad7616.h>

AD7616::AD7616(Embedded_SPI *dev) {
  _dev = dev;
}


uint32_t  AD7616::getMeasurementPair(int cs, uint8_t channel){
  return 0b00000000000000011000000000000000;
}