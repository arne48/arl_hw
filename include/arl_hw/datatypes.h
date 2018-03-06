#ifndef ARL_HW_DATATYPES_H
#define ARL_HW_DATATYPES_H

#include <string>

namespace arl_datatypes {
  /**
   * Conventional structure of hardware state read from implementations of ARL communication device interface
   */
  struct device_data_t {
    int status; //Here will be something after finding a convention
  };

  /**
   * Conventional structure of hardware state which will be written to implementations of ARL communication device interface
   */
  struct device_command_t {
    int status; //Here will be something after finding a convention
  };

  /**
   * Conventional structure for storing muscle information from parameter server
   */
  struct muscle_description_t {
    std::string name;
    double initial_value;
    std::pair<int, int> controller_port_activation;
    std::pair<int, int> controller_port_pressure;
    std::pair<int, int> controller_port_tension;
  };

  /**
   * Conventional structure for returning the current state of the muscles
   */
  struct muscle_status_data_t {
    std::pair<int, int> controller_port_pressure;
    std::pair<int, int> controller_port_tension;
    double current_pressure;
    double tension;
  };

  /**
   * Conventional structure for passing a muscle activation command combined with it's controller port
   */
  struct muscle_command_data_t {
    std::pair<int, int> controller_port_activation;
    double activation;
  };

  /**
   * Conventional structure for passing the status of a generic analog input
   */
  struct analog_input_status_data_t {
    std::pair<int, int> controller_port_analog_input;
    double analog_reading;
  };


}


#endif //ARL_HW_DATATYPES_H
