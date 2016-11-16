#ifndef COMMUNICATION_DEVICE_H
#define COMMUNICATION_DEVICE_H

#include <arl_hw/datatypes.h>

class CommunicationDevice {

public:
  CommunicationDevice() {};

  virtual ~CommunicationDevice() {};

  virtual arl_datatypes::device_data_t read() = 0;

  virtual bool write(arl_datatypes::device_command_t) = 0;

  virtual bool initialize() = 0;

  virtual bool close() = 0;

};

#endif // COMMUNICATION_DEVICE_H
