
#ifndef DUMMY_H
#define DUMMY_H

#include <arl_hw/communication_device.h>

class Dummy : public CommunicationDevice {

public:
  Dummy();

  ~Dummy();

  virtual bool read();

  virtual bool write();

  virtual bool initialize();

  virtual bool close();

};

#endif // DUMMY_H
