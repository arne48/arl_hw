#ifndef ARL_HW_JETSON_TX1_H
#define ARL_HW_JETSON_TX1_H

#include <vector>
#include <map>
#include <arl_hw/communication_device.h>
#include <arl_hw/jetson_tx1_spi.h>
#include <arl_hw/jetson_tx1_gpio.h>
#include <arl_hw/ad5360.h>
#include <arl_hw/ad7616.h>
#include <arl_hw/ad7730.h>


#define BLOW_OFF 0.0

/**
 * Implementation of CommunicationDevice base class which communicats to hardware using a NVIDIA Jetson TX1
 */
class JetsonTX1 : public CommunicationDevice {
public:
  /**
   * Default Constructor
   */
  JetsonTX1();

  /**
   * Destructor
   */
  ~JetsonTX1();

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
  virtual bool write(std::vector<arl_datatypes::muscle_command_data_t> &command_vec);

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
  AD7730 *_lcell;
  uint8_t _lcell_buffer[32];

  AD5360 *_dac;
  AD7616 *_adc;
  JetsonTX1_SPI *_spi;
  JetsonTX1_GPIO *_gpio;

  std::map<int, std::set<int>> _pressure_ports;
  std::map<int, std::set<int>> _tension_ports;

  int _multiplexer_mapping[16] = {7, 6, 4, 1, 13, 10, 9, 8, 5, 3, 2, 0, 15, 14, 12, 11};
  int _multiplexer_address_bus[4] = {36, 37, 38, 63};

};


#endif //ARL_HW_JETSON_TX1_H
