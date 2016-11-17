#ifndef COMMUNICATION_DEVICE_H
#define COMMUNICATION_DEVICE_H

#include <arl_hw/datatypes.h>

/**
 * Interface of base class for implentations of different communication devices for interacting with robots hardware
 */
class CommunicationDevice {

public:

  /**
   * Default Contructor
   */
  CommunicationDevice() {};

  /**
   * Default Destructor
   */
  virtual ~CommunicationDevice() {};

  /**
   * Reads current robot state from hardware
   * @return current state of hardware
   */
  virtual arl_datatypes::device_data_t read() = 0;

  /**
   * Writes robot command to hardware
   * @param command command to issue to hardware
   * @return success of command
   */
  virtual bool write(arl_datatypes::device_command_t command) = 0;

  /**
   * Initialize communication device
   * @return success of command
   */
  virtual bool initialize() = 0;

  /**
   * Cleanup to close communication device
   * @return success of command
   */
  virtual bool close() = 0;

};

#endif // COMMUNICATION_DEVICE_H
