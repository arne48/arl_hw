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
   * Reads current robot state from hardware on a Linux platform using spidev and sysfs
   * @return current state of hardware
   */
  virtual bool read(std::vector<arl_datatypes::muscle_status_data_t> &muscle_status_vec, std::vector<std::pair<int, int> > pressure_controllers,
                    std::vector<std::pair<int, int> > tension_controllers, std::vector<arl_datatypes::analog_input_status_data_t> &analog_input_status_vec,
                    std::vector<std::pair<int, int> > analog_inputs_controllers);

  /**
   * @param command command to issue to hardware on a Linux platform using spidev and sysfs
   * @return success of command
   */
  virtual bool write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec);

  /**
   * Initialize communication device on a Linux platform using spidev and sysfs
   * @return success of command
   */
  virtual bool initialize(std::vector<std::pair<int, int> > pressure_controllers,
                          std::vector<std::pair<int, int> > tension_controllers,
                          std::vector<std::pair<int, int> > analog_inputs_controllers);

  /**
   * Cleanup and close communication device on a Linux platform using spidev and sysfs
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
  AD7730 *_lcell;
  uint8_t _lcell_buffer[64];

  AD5360 *_dac;
  AD7616 *_adc;
  LinuxPlatform_SPI *_spi;
  LinuxPlatform_GPIO *_gpio;

  std::map<int, std::set<int>> _analog_input_ports;
  std::map<int, std::set<int>> _tension_ports;

  int _gpios[CHIP_NUMBER] = {419, 420, 468, 479, 433, 478};

  int _dac_latch_port = 405;
  int _adc_conversion_port = 471;

};


#endif //ARL_HW_LINUX_PLATFORM_H
