#include <arl_hw/robot.h>

#include <wiringPi.h>

ARLRobot::ARLRobot() {
  initialized = false;
}

ARLRobot::~ARLRobot() {
  close();
}

void ARLRobot::initialize(ros::NodeHandle nh) {

  bool using_rpi;
  nh.param<bool>("/arl_hw/using_rpi", using_rpi, false);

  if (!using_rpi) {
    initialized = true;
    ROS_WARN("using dummy interface");

  }else {

    wiringPiSetup();
    for (int pin = 0; pin < 8; ++pin) {
      pinMode(pin, OUTPUT);
      digitalWrite (pin, 0);
    }

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

  ROS_DEBUG("READ with %f hz", 1 / period.toSec());
}

void ARLRobot::write(const ros::Time& time, const ros::Duration& period) {
  if(!initialized) {
    ROS_WARN("Robot not initialized, no data can be read");
    return;
  }

  ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}