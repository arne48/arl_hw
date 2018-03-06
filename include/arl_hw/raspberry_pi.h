#ifndef ARL_HW_RASPBERRY_PI_H
#define ARL_HW_RASPBERRY_PI_H

#include <vector>
#include <map>
#include <arl_hw/communication_device.h>
#include <arl_hw/raspberry_pi_spi.h>
#include <arl_hw/raspberry_pi_gpio.h>
#include <bcm2835.h>
#include <arl_hw/ad5360.h>
#include <arl_hw/ad7616.h>
#include <arl_hw/ad7730.h>


#define BLOW_OFF_VOLTAGE 0.0

/**
 * Implementation of CommunicationDevice base class which communicates to hardware using a Raspberry Pi
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
  virtual bool read(std::vector<arl_datatypes::muscle_status_data_t> &muscle_status_vec, std::vector<std::pair<int, int> > pressure_controllers,
                    std::vector<std::pair<int, int> > tension_controllers, std::vector<arl_datatypes::analog_input_status_data_t> &analog_input_status_vec,
                    std::vector<std::pair<int, int> > analog_inputs_controllers);

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
                          std::vector<std::pair<int, int> > tension_controllers,
                          std::vector<std::pair<int, int> > analog_inputs_controllers);

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
  uint8_t _lcell_buffer[64];

  AD5360 *_dac;
  AD7616 *_adc;
  RaspberryPi_SPI *_spi;
  RaspberryPi_GPIO *_gpio;

  std::map<int, std::set<int>> _analog_input_ports;
  std::map<int, std::set<int>> _tension_ports;

  int _gpios[16] = {RPI_V2_GPIO_P1_24, RPI_V2_GPIO_P1_26, RPI_V2_GPIO_P1_32, RPI_V2_GPIO_P1_36, RPI_V2_GPIO_P1_07,
                    RPI_V2_GPIO_P1_11, RPI_V2_GPIO_P1_13, RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_31, RPI_V2_GPIO_P1_33,
                    RPI_V2_GPIO_P1_35, RPI_V2_GPIO_P1_37, RPI_V2_GPIO_P1_03, RPI_V2_GPIO_P1_05, RPI_V2_GPIO_P1_08,
                    RPI_V2_GPIO_P1_10};

  int _dac_latch_port = RPI_GPIO_P1_18;
  int _adc_conversion_port = RPI_GPIO_P1_16;

};

#endif //ARL_HW_RASPBERRY_PI_H
