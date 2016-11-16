
#ifndef RASPBERRY_PI_H
#define RASPBERRY_PI_H

#include <arl_hw/communication_device.h>
#include <wiringPi.h>

class RaspberryPi : public CommunicationDevice {

public:
  RaspberryPi();

  ~RaspberryPi();

  virtual arl_datatypes::device_data_t read();

  virtual bool write(arl_datatypes::device_command_t);

  virtual bool initialize();

  virtual bool close();

};

#endif // RASPBERRY_PI_H
