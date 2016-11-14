#ifndef ROBOT_H
#define ROBOT_H


#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <arl_hw/communication_device.h>
#include <arl_hw/raspberry_pi.h>

class ARLRobot : public hardware_interface::RobotHW {
public:
  ARLRobot();
  ~ARLRobot();
  void initialize(ros::NodeHandle nh);
  void close();
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  bool initialized;
  CommunicationDevice* dev;
  ros::Time last_read_;
  ros::Time last_write_;
};

#endif // ROBOT_H
