#ifndef ARL_HW_DATA_H
#define ARL_HW_DATA_H

#include <string>

namespace arl_datatypes{

  struct device_data_t {
    int status; //Here will be something after finding a convention
  };

  struct device_command_t {
    int status; //Here will be something after finding a convention
  };

  struct muscle_description_t {
    std::string name;
    double initial_value;
  };




}


#endif //ARL_HW_DATA_H
