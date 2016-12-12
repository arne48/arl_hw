#ifndef COMMUNICATION_DEVICE_H
#define COMMUNICATION_DEVICE_H

#include <vector>
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
   * Destructor
   */
  virtual ~CommunicationDevice() {};

  /**
   * Reads current robot state from hardware
   * @param status output parameter
   * @return success of command
   */
  virtual bool read(std::vector<arl_datatypes::muscle_status_data_t> &status_vec) = 0;

  /**
   * Writes robot command to hardware
   * @param command command to issue to hardware
   * @return success of command
   */
  virtual bool write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec) = 0;

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
