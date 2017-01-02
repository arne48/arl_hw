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

  if (driver_config.using_raspberry_pi) {

    dev = new RaspberryPi();
    initialized = true;
    ROS_INFO("RPi initialized");
  } else {

    dev = new Dummy();
    initialized = true;
    ROS_INFO("Using dummy interface");
  }

  getConfigurationFromParameterServer(nh);


  for (int i = 0; i < names_.size(); i++) {
    arl_interfaces::MuscleHandle muscle_handle(names_[i], &desired_pressures_[i], &current_pressures_[i], &tensions_[i], &activations_[i]);
    muscle_interface.registerHandle(muscle_handle);
  }

  registerInterface(&muscle_interface);

  //Set all controllers to zero activation
  std::vector<arl_datatypes::muscle_command_data_t> command_vec;
  for (int i = 0; i < names_.size(); i++) {
    arl_datatypes::muscle_command_data_t command;
    command.activation = 0.0;
    command.controller_port_activation = activation_controllers_[i];
    command_vec.push_back(command);
  }
  dev->write(command_vec);

  emergency_halt = false;
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

  std::vector<arl_datatypes::muscle_status_data_t> status;
  dev->read(status);

  ROS_DEBUG("READ with %f hz", 1 / period.toSec());
}

void ARLRobot::write(const ros::Time &time, const ros::Duration &period) {
  if (!initialized) {
    ROS_WARN("Robot not initialized, no data can be read");
    return;
  }

  std::vector<arl_datatypes::muscle_command_data_t> command_vec;

  for (int i = 0; i < names_.size(); i++) {
    if (activations_[i] != last_activations_[i]) {
      arl_datatypes::muscle_command_data_t command;
      command.activation = activations_[i];
      command.controller_port_activation = activation_controllers_[i];
      last_activations_[i] = activations_[i];
      command_vec.push_back(command);
    }
  }

  dev->write(command_vec);

  ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}

void ARLRobot::getConfigurationFromParameterServer(ros::NodeHandle nh) {

  ROS_DEBUG("Reading configuration");

  nh.param<bool>("/using_raspberry_pi", driver_config.using_raspberry_pi, false);

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
      activations_.push_back(0.0);

      activation_controllers_.push_back({muscle_list[i]["activation_controller_port"], muscle_list[i]["activation_controller_channel"]});
      pressure_controllers_.push_back({muscle_list[i]["pressure_controller_port"], muscle_list[i]["pressure_controller_channel"]});
      tension_controllers_.push_back({muscle_list[i]["tension_controller_port"], muscle_list[i]["tension_controller_channel"]});


      last_activations_.push_back(0.0);
    }
    catch (...) {
      ROS_ERROR("Unable to parse muscle information");
    }

  }
}

void ARLRobot::executeEmergencyHalt() {
  for (int i = 0; i < names_.size(); i++) {
    dev->emergency_halt(activation_controllers_[i]);
  }
}