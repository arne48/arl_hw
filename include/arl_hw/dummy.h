
#ifndef DUMMY_H
#define DUMMY_H

#include <vector>
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
  virtual bool read(std::vector<arl_datatypes::muscle_status_data_t> &status, std::vector<std::pair<int, int> > pressure_controllers,
                    std::vector<std::pair<int, int> > tension_controllers);

  /**
   * Dummy write
   * @param command
   * @return allways true
   */
  virtual bool write(std::vector<arl_datatypes::muscle_command_data_t> &command);

  /**
   * Dummy initialize
   * @return allways true
   */
  virtual bool initialize(std::vector<std::pair<int, int> > pressure_controllers,
                          std::vector<std::pair<int, int> > tension_controllers);

  /**
   * Dummy close
   * @return allways true
   */
  virtual bool close();

  /**
   * Blows off air from muscle
   */
  virtual void emergency_stop(std::pair<int, int> muscle);

  /**
 * Resets muscle and blows off air
 */
  virtual void reset_muscle(std::pair<int, int> muscle);

};

#endif // DUMMY_H
