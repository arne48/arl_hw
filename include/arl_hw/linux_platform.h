#ifndef ARL_HW_LINUX_PLATFORM_H
#define ARL_HW_LINUX_PLATFORM_H

#include <vector>
#include <map>
#include <arl_hw/communication_device.h>
#include <arl_hw/linux_platform_spi.h>
#include <arl_hw/linux_platform_gpio.h>
#include <arl_hw/ad5360.h>
#include <arl_hw/ad7616.h>
#include <arl_hw/ad7730.h>


#define BLOW_OFF 0.0

/**
 * Implementation of CommunicationDevice base class which communicates to hardware using a spidev and sysfs
 */
class LinuxPlatform : public CommunicationDevice {
public:
  /**
   * Default Constructor
   */
  LinuxPlatform();

  /**
   * Destructor
   */
  ~LinuxPlatform();

  /**
   * Reads current robot state from hardware on a NVIDIA Jetson TX1
   * @return current state of hardware
   */
  virtual bool read(std::vector<arl_datatypes::muscle_status_data_t> &status_vec, std::vector<std::pair<int, int> > pressure_controllers,
                    std::vector<std::pair<int, int> > tension_controllers);

  /**
   * @param command command to issue to hardware on a NVIDIA Jetson TX1
   * @return success of command
   */
  virtual bool write(std::vector<arl_datatypes::muscle_command_data_t> &command);

  /**
   * Initialize communication device on a NVIDIA Jetson TX1
   * @return success of command
   */
  virtual bool initialize(std::vector<std::pair<int, int> > pressure_controllers,
                          std::vector<std::pair<int, int> > tension_controllers);

  /**
   * Cleanup and close communication device on a NVIDIA Jetson TX1
   * @return success of command
   */
  virtual bool close();

  /**
   * Blows off air from muscle
   * @param muscle
   */
  virtual void emergency_stop(std::pair<int, int> muscle);

  /**
   * Resets muscle and blows off air
   * @param muscle
   */
  virtual void reset_muscle(std::pair<int, int> muscle);

private:
//  AD7730 *_lcell;
  uint8_t _lcell_buffer[32];

//  AD5360 *_dac;
//  AD7616 *_adc;
//  LinuxPlatform_SPI *_spi;
//  LinuxPlatform_GPIO *_gpio;

  std::map<int, std::set<int>> _pressure_ports;
  std::map<int, std::set<int>> _tension_ports;

  int _gpios[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};

  int _dac_latch_port = 0;
  int _adc_conversion_port = 0;

};


#endif //ARL_HW_LINUX_PLATFORM_H
