#ifndef COMMUNICATION_DEVICE_H
#define COMMUNICATION_DEVICE_H


class CommunicationDevice {

public:
  CommunicationDevice() {};

  virtual ~CommunicationDevice() {};

  virtual bool read() = 0;

  virtual bool write() = 0;

  virtual bool initialize() = 0;

  virtual bool close() = 0;

};

#endif // COMMUNICATION_DEVICE_H
