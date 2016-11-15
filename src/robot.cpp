#include <arl_hw/robot.h>

ARLRobot::ARLRobot() {
  initialized = false;

  arl_interfaces::MuscleHandle muscle_handle(names_, desired_pressures_, current_pressures_, tensions_);
  muscle_interface.registerHandle(muscle_handle);
  registerInterface(&muscle_interface);

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
  nh.param<bool>("/arl_hw/using_rpi", using_rpi, false);

  if (!using_rpi) {

    dev = new Dummy();
    initialized = true;

    ROS_INFO("Using dummy interface");

  } else {

    dev = new RaspberryPi();

    initialized = true;
    ROS_INFO("RPi initialized");
  }

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

  arl_datatypes::device_data data = dev->read();

  ROS_DEBUG("READ with %f hz", 1 / period.toSec());
}

void ARLRobot::write(const ros::Time &time, const ros::Duration &period) {
  if (!initialized) {
    ROS_WARN("Robot not initialized, no data can be read");
    return;
  }

  arl_datatypes::device_command command;
  dev->write(command);

  ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}
