
#ifndef RASPBERRY_PI_H
#define RASPBERRY_PI_H

#include <arl_hw/communication_device.h>
#include <wiringPi.h>

/**
 * Implementation of CommunicationDevice base class which communicats to hardware using a Raspberry Pi
 */
class RaspberryPi : public CommunicationDevice {

public:

  /**
   * Default Constructor
   */
  RaspberryPi();

  /**
   * Destructor
   */
  ~RaspberryPi();

  /**
   * Reads current robot state from hardware on a Raspberry Pi
   * @return current state of hardware
   */
  virtual arl_datatypes::device_data_t read();

  /**
   * @param command command to issue to hardware on a Raspberry Pi
   * @return success of command
   */
  virtual bool write(arl_datatypes::device_command_t command);

  /**
   * Initialize communication device on a Raspberry Pi
   * @return success of command
   */
  virtual bool initialize();

  /**
   * Cleanup and close communication device on a Raspberry Pi
   * @return success of command
   */
  virtual bool close();

};

#endif // RASPBERRY_PI_H
