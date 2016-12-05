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

  bool using_rpi;
  nh.param<bool>("/using_raspberry_pi", using_rpi, false);

  if (!using_rpi) {

    dev = new Dummy();
    initialized = true;

    ROS_INFO("Using dummy interface");

  } else {

    dev = new RaspberryPi();

    initialized = true;
    ROS_INFO("RPi initialized");
  }

  getMuscleDescriptionsFromParameterServer(nh);


  for(int i=0; i < names_.size(); i++){
    arl_interfaces::MuscleHandle muscle_handle(names_[i], &desired_pressures_[i], &current_pressures_[i], &tensions_[i]);
    muscle_interface.registerHandle(muscle_handle);
  }

  registerInterface(&muscle_interface);

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

  arl_datatypes::device_data_t data = dev->read();

  ROS_DEBUG("READ with %f hz", 1 / period.toSec());
}

void ARLRobot::write(const ros::Time &time, const ros::Duration &period) {
  if (!initialized) {
    ROS_WARN("Robot not initialized, no data can be read");
    return;
  }

  arl_datatypes::device_command_t command;
  dev->write(command);

  //ROS_ERROR("Driver: %s %f",names_[0].c_str(), desired_pressures_[0]);
  //ROS_ERROR("Driver: %s %f",names_[1].c_str(), desired_pressures_[1]);

  ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}

//FIXME loading from parameter server into datastructure not working
void ARLRobot::getMuscleDescriptionsFromParameterServer(ros::NodeHandle nh) {

  ROS_DEBUG("Reading muscle descriptions");

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
    }
    catch (...) {
      ROS_ERROR("Unable to parse muscle information");
    }
  }
}
