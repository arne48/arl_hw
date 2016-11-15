#ifndef ROBOT_H
#define ROBOT_H


#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <arl_hw/datatypes.h>
#include <arl_hw/communication_device.h>
#include <arl_hw/raspberry_pi.h>
#include <arl_hw/dummy.h>
#include <arl_interfaces/muscle_interface.h>
#include <arl_controllers/muscle_controller.h>

class ARLRobot : public hardware_interface::RobotHW {
public:
  ARLRobot();

  ~ARLRobot();

  void initialize(ros::NodeHandle nh);

  void close();

  void read(const ros::Time &time, const ros::Duration &period);

  void write(const ros::Time &time, const ros::Duration &period);

private:
  arl_interfaces::MuscleInterface muscle_interface;
  bool initialized;
  CommunicationDevice *dev;
  ros::Time last_read_;
  ros::Time last_write_;
  std::vector<std::string> names_;
  std::vector<double> desired_pressures_;
  std::vector<double> current_pressures_;
  std::vector<double> tensions_;

};

#endif // ROBOT_H
