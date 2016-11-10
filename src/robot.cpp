#include <arl_hw/robot.h>

ARLRobot::ARLRobot() {}

ARLRobot::~ARLRobot(){}

void ARLRobot::read(const ros::Time& time, const ros::Duration& period){
  ROS_DEBUG("READ with %f hz", 1 / period.toSec());
}

void ARLRobot::write(const ros::Time& time, const ros::Duration& period){
  ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}