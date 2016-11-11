#include <ros/console.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class ARLRobot : public hardware_interface::RobotHW {
public:
  ARLRobot();
  ~ARLRobot();
  void initialize();
  void close();
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  bool initialized;
  ros::Time last_read_;
  ros::Time last_write_;
};