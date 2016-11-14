#include <arl_hw/robot.h>

ARLRobot::ARLRobot() {
  initialized = false;
  
}

ARLRobot::~ARLRobot() {
  if(!initialized) {
    ROS_WARN("Robot not initialized, nothing to close");
    return;
  }
  
  dev->close();
}

void ARLRobot::initialize(ros::NodeHandle nh) {

  bool using_rpi;
  nh.param<bool>("/arl_hw/using_rpi", using_rpi, true);

  if (!using_rpi) {
    initialized = true;
    ROS_WARN("using dummy interface");

  }else {
	  
	dev = new RaspberryPi();

    initialized = true;
    ROS_INFO("RPi initialized");
  }

}

void ARLRobot::close() {
  ROS_INFO("RPi de-initialized");
}

void ARLRobot::read(const ros::Time& time, const ros::Duration& period) {
  if(!initialized) {
    ROS_WARN("Robot not initialized, no data can be read");
    return;
  }
  
  dev->read();

  ROS_DEBUG("READ with %f hz", 1 / period.toSec());
}

void ARLRobot::write(const ros::Time& time, const ros::Duration& period) {
  if(!initialized) {
    ROS_WARN("Robot not initialized, no data can be read");
    return;
  }

  dev->write();

  ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}
