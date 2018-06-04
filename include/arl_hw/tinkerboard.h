#ifndef ARL_HW_TINKERBOARD_H
#define ARL_HW_TINKERBOARD_H

#include <vector>
#include <map>
#include <arl_hw/communication_device.h>
#include <arl_hw/tinkerboard_spi.h>
#include <arl_hw/tinkerboard_gpio.h>
#include <tinkerboard_io.h>
#include <arl_hw/ad5360.h>
#include <arl_hw/ad7616.h>
#include <arl_hw/ad7730.h>

/**
 * Implementation of CommunicationDevice base class which communicates to hardware using a Asus Tinkerboard
 */
class TinkerBoard : public CommunicationDevice {
public:
  /**
   * Default Constructor
   */
  TinkerBoard();

  /**
   * Destructor
   */
  ~TinkerBoard();

  /**
   * Reads current robot state from hardware on a Asus Tinkerboard
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
  TinkerBoard_SPI *_spi;
  TinkerBoard_GPIO *_gpio;

  std::map<int, std::set<int>> _analog_input_ports;
  std::map<int, std::set<int>> _tension_ports;

  int _gpios[CHIP_NUMBER] = {36, 40, 7, 13, 38, 37};

  int _dac_latch_port = 35;
  int _adc_conversion_port = 11;

};

#endif //ARL_HW_TINKERBOARD_H
