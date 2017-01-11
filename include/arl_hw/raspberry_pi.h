#ifndef RASPBERRY_PI_H
#define RASPBERRY_PI_H

#include <vector>
#include <map>
#include <arl_hw/communication_device.h>
#include <arl_hw/raspberry_pi_spi.h>
#include <bcm2835.h>
#include <arl_hw/ad5360.h>
#include <arl_hw/ad7616.h>
#include <arl_hw/ad7730.h>


#define BLOW_OFF 0.0

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
  virtual bool read(std::vector<arl_datatypes::muscle_status_data_t> &status_vec, std::vector<std::pair<int, int> > pressure_controllers,
                    std::vector<std::pair<int, int> > tension_controllers);

  /**
   * @param command command to issue to hardware on a Raspberry Pi
   * @return success of command
   */
  virtual bool write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec);

  /**
   * Initialize communication device on a Raspberry Pi
   * @return success of command
   */
  virtual bool initialize(std::vector<std::pair<int, int> > pressure_controllers,
                          std::vector<std::pair<int, int> > tension_controllers);

  /**
   * Cleanup and close communication device on a Raspberry Pi
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
  uint8_t _lcell_buffer[32];

  AD5360 *_dac;
  AD7616 *_adc;
  RaspberryPi_SPI *_spi;

  std::map<int, std::set<int>> _pressure_ports;
  std::map<int, std::set<int>> _tension_ports;

  int _gpios[8] = {RPI_GPIO_P1_24, RPI_GPIO_P1_26, RPI_V2_GPIO_P1_32, RPI_V2_GPIO_P1_36,
                  RPI_GPIO_P1_07, RPI_GPIO_P1_11, RPI_GPIO_P1_13, RPI_GPIO_P1_15};

};

#endif // RASPBERRY_PI_H
