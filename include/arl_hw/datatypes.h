#ifndef ARL_HW_DATA_H
#define ARL_HW_DATA_H

#include <string>

namespace arl_datatypes{

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

  struct muscle_status_data_t {
    std::pair<int, int> controller_port_pressure;
    std::pair<int, int> controller_port_tension;
    double current_pressure;
    double tension;
  };

  struct muscle_command_data_t {
    std::pair<int, int> controller_port_activation;
    double activation;
  };




}


#endif //ARL_HW_DATA_H
