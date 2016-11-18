
#ifndef DUMMY_H
#define DUMMY_H

#include <arl_hw/communication_device.h>

/**
 * Dummy implementation of CommunicationDevice base class which doesn't execute real communication
 */
class Dummy : public CommunicationDevice {

public:

  /**
   * Default Constructor
   */
  Dummy();

  /**
   * Destructor
   */
  ~Dummy();

  /**
   * Dummy read
   * @return allways true
   */
  virtual arl_datatypes::device_data_t read();

  /**
   * Dummy write
   * @param command
   * @return allways true
   */
  virtual bool write(arl_datatypes::device_command_t command);

  /**
   * Dummy initialize
   * @return allways true
   */
  virtual bool initialize();

  /**
   * Dummy close
   * @return allways true
   */
  virtual bool close();

};

#endif // DUMMY_H
