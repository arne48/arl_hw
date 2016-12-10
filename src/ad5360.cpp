#include <arl_hw/ad5360.h>

AD5360::AD5360(int sync,int ldac, Embedded_SPI *dev) {

  _dev = dev;
  _sync = sync;
  _ldac = ldac;

  _vref[0] = 3.0;
  _vref[1] = 3.0;

  //_dev->setGPIO(_sync, EMBD_SPI_HIGH);

  //Make registers transparent
  _dev->setGPIO(_ldac, EMBD_SPI_LOW);

}

void AD5360::writeDAC(AD5360_bank_t bank, AD5360_ch_t ch, unsigned int data){
  AD5360::write(DAC, bank, ch, data);
}

unsigned int AD5360::getDAC(AD5360_bank_t bank, AD5360_ch_t ch){
  return _dac[bank][ch];
}

void AD5360::setVoltage(AD5360_bank_t bank, AD5360_ch_t ch, double voltage){
  unsigned int data = AD5360::voltageToDAC(bank, ch, voltage);
  AD5360::writeDAC(bank, ch, data);
}

void AD5360::writeOffset(AD5360_bank_t bank, AD5360_ch_t ch, unsigned int data){
  AD5360::write(OFFSET, bank, ch, data);
  AD5360::latchDAC();
}

unsigned int AD5360::getOffset(AD5360_bank_t bank, AD5360_ch_t ch){
  return _offset[bank][ch];
}

void AD5360::writeGain(AD5360_bank_t bank, AD5360_ch_t ch, unsigned int data){
  AD5360::write(OFFSET, bank, ch, data);
  AD5360::latchDAC();
}

unsigned int AD5360::getGain(AD5360_bank_t bank, AD5360_ch_t ch){
  return _gain[bank][ch];
}

void AD5360::latchDAC(){
  _dev->setGPIO(_ldac, EMBD_SPI_LOW);

  _dev->setGPIO(_ldac,EMBD_SPI_HIGH);
}

void AD5360::writeGlobalOffset(AD5360_bank_t bank, unsigned int data){

  unsigned long cmd = 0;
  data = data & 0x3FFF; 	// 14-bit mask
  switch (bank) {
    case BANK0:
      cmd = (cmd | AD5360_WRITE_OFS0 | data);
      _globalOffset[0] = data;
      break;
    case BANK1:
      cmd = (cmd | AD5360_WRITE_OFS1 | data);
      _globalOffset[1] = data;
      break;
    default:
      return;
  }

  AD5360::writeCommand(cmd);

}

unsigned int AD5360::getGlobalOffset(AD5360_bank_t bank){
  return _globalOffset[bank];
}

void AD5360::setGlobalVref(AD5360_bank_t bank, double voltage){
  switch (bank) {
    case BANK0:
      _vref[0] = voltage;
      break;
    case BANK1:
      _vref[1] = voltage;
      break;
    default:
      break;
  }
}

double AD5360::getGlobalVref(AD5360_bank_t bank){
  return _vref[bank];
}

void AD5360::writeCommand(unsigned long cmd){

  //_dev->setGPIO(_sync, EMBD_SPI_LOW);

  unsigned char data[3];
  data[0] = (cmd >> 16) & 0xFF;
  data[1] = (cmd >> 8) & 0xFF;
  data[2] = cmd & 0xFF;
  _dev->transferSPI(3, data);

  //_dev->setGPIO(_sync, EMBD_SPI_HIGH);

}

void AD5360::write(AD5360_reg_t reg, AD5360_bank_t bank, AD5360_ch_t ch, unsigned int data){
  data = data & AD5360_DATA_MASK;

  unsigned int  (*localData)[2][AD5360_MAX_CHANNELS];

  unsigned long cmd = 0;

  switch (reg) {
    case DAC:
      cmd = cmd | AD5360_WRITE_DAC;
      localData = &_dac;
      break;
    case OFFSET:
      cmd = cmd | AD5360_WRITE_OFFSET;
      localData = &_offset;
      break;
    case GAIN:
      cmd = cmd | AD5360_WRITE_GAIN;
      localData = &_gain;
      break;
    default:
      return;
  }

  if (ch > AD5360_MAX_CHANNELS && ch != CHALL){
    return;
  }

  if (ch == CHALL){
    switch (bank){
      case BANK0:
        cmd = cmd | AD5360_ALL_BANK0;
        for (int c = 0; c < AD5360_MAX_CHANNELS; c++){
          (*localData)[0][c] = data;
        }
        break;

      case BANK1:
        cmd = cmd | AD5360_ALL_BANK1;
        for (int c = 0; c < AD5360_MAX_CHANNELS; c++){
          (*localData)[0][c] = data;
        }
        break;

      case BANKALL:
        for (int c = 0; c < AD5360_MAX_CHANNELS; c++){
          (*localData)[0][c] = data;
          (*localData)[1][c] = data;
        }
        break;

      default:
        return;
    }
  } else {

    switch (bank){
      case BANK0:
        cmd = cmd | AD5360_BANK0 | ((unsigned long)ch << 16);
        (*localData)[0][ch] = data;
        break;

      case BANK1:
        cmd = cmd | AD5360_BANK1 | ((unsigned long) ch << 16);
        (*localData)[1][ch] = data;
        break;

      default:
        return;
    }
  }

  cmd = cmd | data;
  AD5360::writeCommand(cmd);
}

unsigned int AD5360::voltageToDAC(AD5360_bank_t bank, AD5360_ch_t ch, double voltage){

  double mm = (double)(_gain[bank][ch] + 1)/pow(2,16);
  double cc = (double) _offset[bank][ch] - 0x8000;
  double d = voltage*pow(2,16)/(4*_vref[bank]) + 4*(double)_offset[bank][ch];
  unsigned int data = (unsigned int)((d - cc)/mm);
  return data;

}