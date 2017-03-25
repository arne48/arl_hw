#include <arl_hw/robot.h>

ARLRobot::ARLRobot() {
  initialized = false;
}

ARLRobot::~ARLRobot() {
  if (!initialized) {
    ROS_WARN("Robot not initialized, nothing to close");
    return;
  }

  dev->close();
}

void ARLRobot::initialize(ros::NodeHandle nh) {

  getConfigurationFromParameterServer(nh);

  if (driver_config.using_raspberry_pi) {
    dev = new RaspberryPi();
    initialized = true;
    ROS_INFO("RPi initialized");
  } else if(driver_config.using_jetson_tx1) {
    dev = new JetsonTX1;
    initialized = true;
    ROS_INFO("NVIDIA Jetson TX1 initialized");
  }
  else {
    dev = new Dummy();
    initialized = true;
    ROS_INFO("Using dummy interface");
  }

  dev->initialize(pressure_controllers_, tension_controllers_);


  for (unsigned int i = 0; i < names_.size(); i++) {
    arl_interfaces::MuscleHandle muscle_handle(names_[i], &desired_pressures_[i], &current_pressures_[i], &tensions_[i], &activations_[i]);
    muscle_interface.registerHandle(muscle_handle);
  }

  registerInterface(&muscle_interface);

  //Set all controllers to blow-off activation
  std::vector<arl_datatypes::muscle_command_data_t> command_vec;
  for (unsigned int i = 0; i < names_.size(); i++) {
    arl_datatypes::muscle_command_data_t command;
    command.activation = -1.0;
    command.controller_port_activation = activation_controllers_[i];
    command_vec.push_back(command);
  }
  dev->write(command_vec);

  status.reserve(names_.size());

  emergency_stop = false;
}

void ARLRobot::close() {
  dev->close();

  ROS_INFO("Device uninitialized");
}

void ARLRobot::read(const ros::Time &time, const ros::Duration &period) {
  if (!initialized) {
    ROS_WARN("Robot not initialized, no data can be read");
    return;
  }

  dev->read(status, pressure_controllers_, tension_controllers_);

  /* Status of controllers should be in the same order as in the internal
   * datastructure. To ensure this the address (controller port & channel)
   * of the pressure controller of each muscle is used as identifier.
   * If this check fails the address is searched in the remaining muscles.
   */
  if(status.size() == names_.size()){
    for (unsigned int i = 0; i < status.size(); i++) {
      unsigned int size = pressure_controllers_.size();
      for (unsigned int j = 0; j < size; j++) {
        int guessed_id = (i + j) % size;
        if (pressure_controllers_[guessed_id].first == status[i].controller_port_pressure.first &&
            pressure_controllers_[guessed_id].second == status[i].controller_port_pressure.second) {
          current_pressures_[guessed_id] = status[i].current_pressure;
          tensions_[guessed_id] = status[i].tension;
          break;
        }
      }
    }
  } else {
    ROS_ERROR("%d muscles are registered on robot but the status of %d were read from hardware", int(names_.size()), int(status.size()));
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
    if (activations_[i] != last_activations_[i]) {
      arl_datatypes::muscle_command_data_t command;
      command.activation = activations_[i];
      command.controller_port_activation = activation_controllers_[i];
      last_activations_[i] = activations_[i];
      command_vec.push_back(command);
    }
  }

  dev->write(command_vec);

  //ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}

void ARLRobot::getConfigurationFromParameterServer(ros::NodeHandle nh) {

  ROS_DEBUG("Reading configuration");

  nh.param<bool>("/using_raspberry_pi", driver_config.using_raspberry_pi, false);

  nh.param<bool>("/using_jetson_tx1", driver_config.using_jetson_tx1, false);

  nh.param<bool>("/publish_every_rt_jitter", driver_config.publish_every_rt_jitter, false);

  nh.param<bool>("/halt_on_slow_rt_loop", driver_config.halt_on_slow_rt_loop, false);

  nh.param<double>("/min_acceptable_rt_loop_frequency", driver_config.min_acceptable_rt_loop_frequency, 1000.0);


  XmlRpc::XmlRpcValue muscle_list;
  nh.getParam("/muscle_list", muscle_list);
  if (muscle_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("Parameter muscle_list should be an array");
    return;
  } else {
    ROS_DEBUG("Muscle list of size %d found", muscle_list.size());
  }


  /* define all muscles */
  for (int i = 0; i < muscle_list.size(); ++i) {
    if (!muscle_list[i].hasMember("name") || !muscle_list[i].hasMember("initial_value")) {
      ROS_ERROR("Either name or initial value not defined for %d muscle", i);
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
      activations_.push_back(-1.0);

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
}

void ARLRobot::executeEmergencyStop() {
  for (unsigned int i = 0; i < names_.size(); i++) {
    dev->emergency_stop(activation_controllers_[i]);
    desired_pressures_[i] = 0.0;
    activations_[i] = -1.0;
  }
}

void ARLRobot::resetMuscles() {
  for (unsigned int i = 0; i < names_.size(); i++) {
    dev->reset_muscle(activation_controllers_[i]);
    desired_pressures_[i] = 0.0;
    activations_[i] = -1.0;
  }
}

void ARLRobot::resetMuscle(std::string name) {
  for (unsigned int i = 0; i < names_.size(); i++) {
    if (names_[i] == name) {
      dev->reset_muscle(activation_controllers_[i]);
      desired_pressures_[i] = 0.0;
      activations_[i] = -1.0;
    }
  }
}
