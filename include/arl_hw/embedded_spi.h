#ifndef ARL_HW_EMBEDDED_SPI_H
#define ARL_HW_EMBEDDED_SPI_H

#define EMBD_SPI_HIGH true
#define EMBD_SPI_LOW  false


class Embedded_SPI {

public:
  Embedded_SPI(){};

  virtual ~Embedded_SPI(){};

  virtual bool transferSPI(int cs, int data_len, char data[]) = 0;

};


#endif //ARL_HW_EMBEDDED_SPI_H
