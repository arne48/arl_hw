/*
 * Thanks to JQIamo and their repo AD536x-arduino
 */

#ifndef AD5360_h
#define AD5360_h

#include <math.h>
#include <arl_hw/embedded_spi.h>

#define AD5360_MAX_CHANNELS 8
#define AD5360_RESOLUTION 16
#define AD5360_DATA_MASK 0xFFFF
#define AD5360_CH_MASK 0x07

#define AD5360_DEFAULT_DAC 0x8000
#define AD5360_DEFAULT_OFFSET 0x8000
#define AD5360_DEFAULT_GAIN 0xFFFF
#define AD5360_DEFAULT_GLOBALOFFSET 0x2000

#define AD5360_DEFAULT_MAX 0xFFFF
#define AD5360_DEFAULT_MIN 0x0000

#define AD5360_SPECIAL_FUNCTION 0
#define	AD5360_WRITE_DAC      3UL << 22
#define AD5360_WRITE_OFFSET 	2UL << 22
#define	AD5360_WRITE_GAIN 		1UL << 22

#define AD5360_BANK0		1UL << 19
#define AD5360_BANK1		2UL << 19
#define AD5360_ALL_DACS		0
#define AD5360_ALL_BANK0	1UL << 16
#define AD5360_ALL_BANK1	2UL << 16

#define AD5360_NOP	0
#define AD5360_WR_CR 1UL << 15

#define AD5360_X1B 4
#define AD5360_X1A 0
#define AD5360_T_SHTDWN_EN 2
#define AD5360_T_SHTDWN_DIS 0
#define AD5360_SOFT_PWR_UP 1
#define AD5360_SOFT_PWR_DWN 0
#define AD5360_WRITE_OFS0 2UL << 16
#define AD5360_WRITE_OFS1 3UL << 16


// register types
enum AD5360_reg_t { DAC, OFFSET, GAIN };

// Bank types
enum AD5360_bank_t { BANK0, BANK1, BANKALL };

// channel types
enum AD5360_ch_t { CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7, CHALL };

class AD5360 {
public:

  AD5360(int sync, int ldac, Embedded_SPI *dev);

  void writeDAC(AD5360_bank_t bank, AD5360_ch_t ch, unsigned int data);

  unsigned int getDAC(AD5360_bank_t bank, AD5360_ch_t ch);

  void writeOffset(AD5360_bank_t bank, AD5360_ch_t ch, unsigned int data);
  unsigned int getOffset(AD5360_bank_t bank, AD5360_ch_t ch);

  void writeGain(AD5360_bank_t bank, AD5360_ch_t ch, unsigned int data);
  unsigned int getGain(AD5360_bank_t bank, AD5360_ch_t ch);

  void setVoltage(AD5360_bank_t bank, AD5360_ch_t ch, double voltage);

  void latchDAC();

  void writeGlobalOffset(AD5360_bank_t bank, unsigned int data);
  unsigned int getGlobalOffset(AD5360_bank_t bank);

  void setGlobalVref(AD5360_bank_t bank, double voltage);
  double getGlobalVref(AD5360_bank_t bank);

  void writeCommand(unsigned long cmd);


private:

  int _sync, _ldac;

  Embedded_SPI *_dev;

  unsigned int _dac[2][AD5360_MAX_CHANNELS];

  unsigned int _offset[2][AD5360_MAX_CHANNELS];

  unsigned int _gain[2][AD5360_MAX_CHANNELS];

  unsigned int _globalOffset[2];

  double _vref[2];

  void write(AD5360_reg_t reg, AD5360_bank_t bank, AD5360_ch_t ch, unsigned int data);

  unsigned int voltageToDAC(AD5360_bank_t bank, AD5360_ch_t ch, double voltage);

};



#endif