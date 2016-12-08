#ifndef RASPBERRY_PI_H
#define RASPBERRY_PI_H

#include <vector>
#include <arl_hw/communication_device.h>
#include <arl_hw/raspberry_pi_spi.h>
#include <arl_hw/ad5360.h>
#include <wiringPi.h>

/**
 * Implementation of CommunicationDevice base class which communicats to hardware using a Raspberry Pi
 */
class RaspberryPi : public CommunicationDevice{

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
  virtual bool read(std::vector<arl_datatypes::muscle_status_data_t> &status);

  /**
   * @param command command to issue to hardware on a Raspberry Pi
   * @return success of command
   */
  virtual bool write(std::vector<arl_datatypes::muscle_command_data_t> &command);

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

private:

  std::vector<AD5360*> _adc_vec;
  RaspberryPi_SPI *_spi;



};

#endif // RASPBERRY_PI_H
