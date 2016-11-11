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
    pinMode(0, OUTPUT);
    digitalWrite (0, LOW);

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

  digitalWrite (0, HIGH);
  delay(10);

  ROS_DEBUG("READ with %f hz", 1 / period.toSec());
}

void ARLRobot::write(const ros::Time& time, const ros::Duration& period) {
  if(!initialized) {
    ROS_WARN("Robot not initialized, no data can be read");
    return;
  }

  digitalWrite (0, LOW);
  delay(10);
  
  ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}