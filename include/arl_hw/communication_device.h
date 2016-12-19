#ifndef COMMUNICATION_DEVICE_H
#define COMMUNICATION_DEVICE_H

#include <vector>
#include <arl_hw/datatypes.h>

/**
 * Important: The driver addresses controllers by the scheme [port(0-7):channel(0-15)]
 * this has to be taken into account.
 *
 * Interface of base class for implementations of different communication devices for interacting with robots hardware
 * It is assumed that every controller has 16 channels which are numbered 0-15.
 * The remapping of this to the eventual division into several banks or groups e.g. [0-15 (A0-7:0-7 ; B0-7:8-15)]
 * has to be performed by the implementation of this interface.
 * Also the mapping of the Chip-Select ids (0-7) to their actual values for the used hardware has to be
 * done by the implementing class.
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
   * @param status_vec output parameter
   * @return success of command
   */
  virtual bool read(std::vector<arl_datatypes::muscle_status_data_t> &status_vec) = 0;

  /**
   * Writes robot command to hardware
   * @param command_vec command to issue to hardware
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
