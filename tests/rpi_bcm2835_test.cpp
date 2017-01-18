#include <bcm2835.h>
#include <stdio.h>

#define MIBI 1024 * 1024

int main(int argc, char **argv) {
  if (!bcm2835_init()) {
    printf("bcm2835_init failed. Are you running as root??\n");
    return 1;
  } if (!bcm2835_spi_begin()) {
    printf("bcm2835_spi_begin failed. Are you running as root??\n");
    return 1;
  }
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS_NONE, LOW);


  //Chip-Selects
  bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_26, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_32, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_36, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_07, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_11, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_13, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(RPI_GPIO_P1_15, BCM2835_GPIO_FSEL_OUTP);

  bcm2835_gpio_write(RPI_GPIO_P1_24, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_26, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_32, HIGH);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_36, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_07, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_11, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_13, HIGH);
  bcm2835_gpio_write(RPI_GPIO_P1_15, HIGH);


  char tx_data[MIBI];
  char rx_data[MIBI];
  for(int i=0; i < MIBI; i++){
    tx_data[i] = i % 256;
  }

  //measure with an oscilloscope that GPIO 24 and 26 are not driven as HW CSs
  bcm2835_gpio_write(RPI_V2_GPIO_P1_32, LOW);
  bcm2835_spi_transfernb(tx_data, rx_data, MIBI);
  bcm2835_gpio_write(RPI_V2_GPIO_P1_32, HIGH);

  int fail_count = 0;
  for(int i=0; i < MIBI; i++){
    if (tx_data[i] != rx_data[i]){
      //printf("Failed at idx %d 0x%02X -> 0x%02X\n", i, tx_data[i], rx_data[i]);
      fail_count++;
    }
  }

  if(fail_count > 0) {
    printf("%d errors occurred", fail_count);
  }

  bcm2835_spi_end();
  bcm2835_close();
  return 0;
}