
#ifndef DUMMY_H
#define DUMMY_H

#include <arl_hw/communication_device.h>

class Dummy : public CommunicationDevice {

public:
  Dummy();

  ~Dummy();

  virtual arl_datatypes::device_data read();

  virtual bool write(arl_datatypes::device_command);

  virtual bool initialize();

  virtual bool close();

};

#endif // DUMMY_H
