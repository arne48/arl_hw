#include <arl_hw/robot.h>

ARLRobot::ARLRobot() {
  initialized = true;
}

ARLRobot::~ARLRobot() {
  if (!initialized) {
    ROS_WARN("Robot not initialized, nothing to close");
    return;
  }

  dev_->close();
}

void ARLRobot::initialize(ros::NodeHandle nh) {

  getConfigurationFromParameterServer(nh);

  bool using_fallback_dummy = false;

  if (driver_config.platform == "raspberry_pi") {

#ifdef FOR_RPI
    dev_ = new RaspberryPi();
    initialized = true;
    ROS_INFO("RaspberryPi initialized");
#else
    ROS_FATAL("Driver was not built with RaspberryPi support");
    using_fallback_dummy = true;
#endif

  } else if(driver_config.platform == "linux_platform") {

#ifdef FOR_LINUX_PLATFORM
    dev_ = new LinuxPlatform();
    initialized = true;
    ROS_INFO("Linux spidev_ and sysfs platform initialized");
#else
    ROS_FATAL("Driver was not built with Linux Platform support");
    using_fallback_dummy = true;
#endif

  } else if(driver_config.platform == "tinkerboard") {

#ifdef FOR_TINKERBOARD
    dev_ = new TinkerBoard();
    initialized = true;
    ROS_INFO("Asus TinkerBoard initialized");
#else
    ROS_FATAL("Driver was not built with Asus Tinkerboard support");
    using_fallback_dummy = true;
#endif

  } else if(driver_config.platform == "dummy") {
    dev_ = new Dummy();
    initialized = true;
    ROS_INFO("Using Dummy interface");
  } else {
    dev_ = new Dummy();
    initialized = true;
    ROS_WARN("Platform name not know using Dummy instead");
  }

  if(using_fallback_dummy) {
    dev_ = new Dummy();
    initialized = true;
    ROS_FATAL("Using Fallback Dummy");
  }

  dev_->initialize(pressure_controllers_, tension_controllers_, analog_input_controllers_);


  for (unsigned int i = 0; i < names_.size(); i++) {
    arl_interfaces::MuscleHandle muscle_handle(names_[i], &desired_pressures_[i], &current_pressures_[i], &tensions_[i],
                                               &activations_[i], &tensions_filtered_[i], &control_modes_[i]);
    muscle_interface.registerHandle(muscle_handle);
  }

  registerInterface(&muscle_interface);

  //Set all controllers to blow-off activation
  std::vector<arl_datatypes::muscle_command_data_t> command_vec;
  for (unsigned int i = 0; i < names_.size(); i++) {
    arl_datatypes::muscle_command_data_t command;
    command.activation = -0.5;
    command.controller_port_activation = activation_controllers_[i];
    command_vec.push_back(command);
  }
  dev_->write(command_vec);

  command_vec.clear();

  for (unsigned int i = 0; i < names_.size(); i++) {
    arl_datatypes::muscle_command_data_t command;
    command.activation = -0.3;
    command.controller_port_activation = activation_controllers_[i];
    command_vec.push_back(command);
  }
  dev_->write(command_vec);

  muscle_status_.reserve(names_.size());
  analog_input_status_.reserve(analog_input_names_.size());

  emergency_stop = false;
}

void ARLRobot::close() {
  dev_->close();

  ROS_INFO("dev_ice uninitialized");
}

void ARLRobot::read(const ros::Time &time, const ros::Duration &period) {
  if (!initialized) {
    ROS_WARN("Robot not initialized, no data can be read");
    return;
  }

  dev_->read(muscle_status_, pressure_controllers_, tension_controllers_,
             analog_input_status_, analog_input_controllers_);

  /* States of controllers should be in the same order as in the internal
   * datastructure. To ensure this the address (controller port & channel)
   * of the pressure controller of each muscle are used as identifier.
   * If this check fails the address is searched in the remaining muscles.
   */
  if (muscle_status_.size() == names_.size()) {
    for (unsigned int i = 0; i < muscle_status_.size(); i++) {
      unsigned int size = pressure_controllers_.size();
      for (unsigned int j = 0; j < size; j++) {
        int guessed_id = (i + j) % size;
        if (pressure_controllers_[guessed_id].first == muscle_status_[i].controller_port_pressure.first &&
            pressure_controllers_[guessed_id].second == muscle_status_[i].controller_port_pressure.second) {
          current_pressures_[guessed_id] = muscle_status_[i].current_pressure;
          tensions_[guessed_id] = muscle_status_[i].tension;
          break;
        }
      }
    }
  } else {
    ROS_ERROR("%d muscles are registered on robot but the status of %d were read from hardware", int(names_.size()), int(muscle_status_.size()));
  }

  if (analog_input_status_.size() == analog_input_names_.size()) {
    for (unsigned int i = 0; i < analog_input_status_.size(); i++) {
      unsigned int size = analog_input_controllers_.size();
      for (unsigned int j = 0; j < size; j++) {
        int guessed_id = (i + j) % size;
        if (analog_input_controllers_[guessed_id].first == analog_input_status_[i].controller_port_analog_input.first &&
            analog_input_controllers_[guessed_id].second == analog_input_status_[i].controller_port_analog_input.second) {
          analog_inputs_[guessed_id] = analog_input_status_[i].analog_reading;
          break;
        }
      }
    }
  } else {
    ROS_ERROR("%d analog inputs are registered on robot but the status of %d were read from hardware",
              int(analog_input_names_.size()), int(analog_input_status_.size()));
  }

  //ROS_DEBUG("READ with %f hz", 1 / period.toSec());
}

void ARLRobot::write(const ros::Time &time, const ros::Duration &period) {
  if (!initialized) {
    ROS_WARN("Robot not initialized, no data can be read");
    return;
  }

  std::vector<arl_datatypes::muscle_command_data_t> command_vec;

  for (unsigned int i = 0; i < names_.size(); i++) {
    //if (activations_[i] != last_activations_[i]) {
    arl_datatypes::muscle_command_data_t command;
    command.activation = activations_[i];
    command.controller_port_activation = activation_controllers_[i];
    //last_activations_[i] = activations_[i];
    command_vec.push_back(command);
    //}
  }

  dev_->write(command_vec);

  //ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}

void ARLRobot::getConfigurationFromParameterServer(ros::NodeHandle nh) {

  ROS_DEBUG("Reading configuration");

  nh.param<std::string>("/platform", driver_config.platform, "dummy");

  nh.param<bool>("/publish_every_rt_jitter", driver_config.publish_every_rt_jitter, false);

  nh.param<bool>("/halt_on_slow_rt_loop", driver_config.halt_on_slow_rt_loop, false);

  nh.param<double>("/min_acceptable_rt_loop_frequency", driver_config.min_acceptable_rt_loop_frequency, 490.0);


  XmlRpc::XmlRpcValue muscle_list;
  nh.getParam("/muscle_list", muscle_list);
  if (muscle_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("Parameter muscle_list should be an array");
    return;
  } else {
    ROS_DEBUG("Muscle list of size %d found", muscle_list.size());
  }

  /* define all muscles */
  for (unsigned int i = 0; i < muscle_list.size(); ++i) {
    if (!muscle_list[i].hasMember("name") || !muscle_list[i].hasMember("initial_value") ||
        !muscle_list[i].hasMember("activation_controller_port") || !muscle_list[i].hasMember("activation_controller_channel") ||
        !muscle_list[i].hasMember("pressure_controller_port") || !muscle_list[i].hasMember("pressure_controller_channel") ||
        !muscle_list[i].hasMember("tension_controller_port") || !muscle_list[i].hasMember("tension_controller_channel")) {
      ROS_ERROR("Definition not complete for muscle %d", i);
      continue;
    }

    try {
      //Try to access all muscle fields first and safe them
      std::string name = std::string(muscle_list[i]["name"]);
      double initial_value = muscle_list[i]["initial_value"];

      ROS_DEBUG("Found on server %s at %f", name.c_str(), initial_value);

      //If all fields where accessible safe a new set of muscle information to the robot hardware
      names_.push_back(name);
      desired_pressures_.push_back(initial_value);
      current_pressures_.push_back(0.0);
      tensions_.push_back(0.0);
      activations_.push_back(-0.3);
      tensions_filtered_.push_back(0.0);
      control_modes_.push_back(arl_hw_msgs::MuscleCommand::CONTROL_MODE_BY_ACTIVATION);

      index_map_[name] = i;

      activation_controllers_.push_back({muscle_list[i]["activation_controller_port"], muscle_list[i]["activation_controller_channel"]});
      pressure_controllers_.push_back({muscle_list[i]["pressure_controller_port"], muscle_list[i]["pressure_controller_channel"]});
      tension_controllers_.push_back({muscle_list[i]["tension_controller_port"], muscle_list[i]["tension_controller_channel"]});

      pressure_ports.insert(muscle_list[i]["pressure_controller_port"]);
      tension_ports.insert(muscle_list[i]["tension_controller_port"]);

      last_activations_.push_back(0.0);
    }
    catch (...) {
      ROS_ERROR("Unable to parse muscle information");
    }
  }

  //Try to access all analog input fields
  XmlRpc::XmlRpcValue analog_inputs;
  nh.getParam("/analog_inputs", analog_inputs);
  if (analog_inputs.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("Parameter analog_inputs should be an array");
    return;
  } else {
    ROS_DEBUG("Analog inputs list of size %d found", analog_inputs.size());
  }

  for (unsigned int i = 0; i < analog_inputs.size(); ++i) {
    if (!analog_inputs[i].hasMember("name") || !analog_inputs[i].hasMember("controller_channel") ||
        !analog_inputs[i].hasMember("controller_port")) {
      ROS_ERROR("Either name, channel or port not defined for analog input %d", i);
      continue;
    }

    try {
      std::string name = std::string(analog_inputs[i]["name"]);
      analog_input_names_.push_back(name);
      analog_input_controllers_.push_back({analog_inputs[i]["controller_port"], analog_inputs[i]["controller_channel"]});
      analog_inputs_.push_back(0.0);
    }
    catch (...) {
      ROS_ERROR("Unable to parse analog inputs information");
    }
  }
}

struct ARLRobot::muscle_info_t ARLRobot::getMuscleInfo(unsigned long index) {
  struct muscle_info_t ret = {
      .name = names_[index],
      .activation = activations_[index],
      .current_pressure = current_pressures_[index],
      .desired_pressure = desired_pressures_[index],
      .tension = tensions_[index],
      .tension_filtered = tensions_filtered_[index],
      .control_mode = control_modes_[index]
  };
  return ret;
}

unsigned long ARLRobot::getNumberOfMuscles() {
  return names_.size();
}

unsigned long ARLRobot::getNumberOfAnalogInputs() {
  return analog_input_names_.size();
}

struct ARLRobot::analog_input_info_t ARLRobot::getAnalogInputInfo(unsigned long index) {
  struct analog_input_info_t ret = {
      .name = analog_input_names_[index],
      .voltage = analog_inputs_[index]
  };
  return ret;
}

void ARLRobot::updateMuscleValues(arl_hw_msgs::MusculatureCommand musculature_command) {

  for (arl_hw_msgs::MuscleCommand command : musculature_command.muscle_commands) {
    unsigned int muscle_index = index_map_.find(command.name)->second;

    if (command.control_mode == arl_hw_msgs::MuscleCommand::CONTROL_MODE_BY_ACTIVATION) {
      control_modes_[muscle_index] = arl_hw_msgs::MuscleCommand::CONTROL_MODE_BY_ACTIVATION;
      desired_pressures_[muscle_index] = 0.0;
      activations_[muscle_index] = command.activation;
    } else if (command.control_mode == arl_hw_msgs::MuscleCommand::CONTROL_MODE_BY_PRESSURE) {
      control_modes_[muscle_index] = arl_hw_msgs::MuscleCommand::CONTROL_MODE_BY_PRESSURE;
      desired_pressures_[muscle_index] = command.pressure;
    }

  }
}

void ARLRobot::executeEmergencyStop() {
  for (unsigned int i = 0; i < names_.size(); i++) {
    dev_->emergency_stop(activation_controllers_[i]);
    desired_pressures_[i] = 0.0;
    activations_[i] = -1.0;
  }
}

void ARLRobot::resetMuscles() {
  for (unsigned int i = 0; i < names_.size(); i++) {
    dev_->reset_muscle(activation_controllers_[i]);
    desired_pressures_[i] = 0.0;
    activations_[i] = -1.0;
  }
}

void ARLRobot::resetMuscle(std::string name) {
  for (unsigned int i = 0; i < names_.size(); i++) {
    if (names_[i] == name) {
      dev_->reset_muscle(activation_controllers_[i]);
      desired_pressures_[i] = 0.0;
      activations_[i] = -1.0;
    }
  }
}
