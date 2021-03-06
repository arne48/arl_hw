#ifndef ARL_HW_COMMUNICATION_DEVICE_H
#define ARL_HW_COMMUNICATION_DEVICE_H

#include <vector>
#include <set>
#include <arl_hw/datatypes.h>

#define CHIP_NUMBER 6
#define BLOW_OFF_VOLTAGE 0.0

/**
 * Important: The driver addresses controllers by the scheme [port(0-7):channel(0-15)]
 * this has to be taken into account.
 *
 * Interface of base class for implementations of different communication devices for interacting with robots hardware
 * It is assumed that every controller has 16 channels which are numbered 0-15.
 * The remapping of this to the eventual division into several banks or groups e.g. [0-15 (A0-7:0-7 ; B0-7:8-15)]
 * has to be performed by the implementation of this interface.
 * Also the mapping of the Chip-Select ids (0-15) to their actual values for the used hardware has to be
 * done by the implementing class.
 */
class CommunicationDevice {
public:
  /**
   * Default Constructor
   */
  CommunicationDevice() {};

  /**
   * Destructor
   */
  virtual ~CommunicationDevice() {};

  /**
   * Reads current robot state from hardware
   * @param status_vec output parameter
   * @param pressure_controllers
   * @param tension_controllers
   * @return success of command
   */
  virtual bool read(std::vector<arl_datatypes::muscle_status_data_t> &muscle_status_vec, std::vector<std::pair<int, int> > pressure_controllers,
                    std::vector<std::pair<int, int> > tension_controllers, std::vector<arl_datatypes::analog_input_status_data_t> &analog_input_status_vec,
                    std::vector<std::pair<int, int> > analog_inputs_controllers) = 0;

  /**
   * Writes robot command to hardware
   * @param command_vec command to issue to hardware
   * @return success of command
   */
  virtual bool write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec) = 0;

  /**
   * Initialize communication device
   * @param pressure_controllers
   * @param tension_controllers
   * @return success of command
   */
  virtual bool initialize(std::vector<std::pair<int, int> > pressure_controllers,
                          std::vector<std::pair<int, int> > tension_controllers,
                          std::vector<std::pair<int, int> > analog_inputs_controllers) = 0;

  /**
   * Cleanup to close communication device
   * @return success of command
   */
  virtual bool close() = 0;


  /**
   * Blows off air from muscle
   * @param muscle port of muscle to stop
   */
  virtual void emergency_stop(std::pair<int, int> muscle) = 0;

  /**
   * Resets muscle and blows off air
   * @param muscle
   */
  virtual void reset_muscle(std::pair<int, int> muscle) = 0;

};

#endif //ARL_HW_COMMUNICATION_DEVICE_H
