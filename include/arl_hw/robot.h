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

/**
 * Implementation of Robot Hardware Interface for ARL's pneumatic muscle driven robots
 */
class ARLRobot : public hardware_interface::RobotHW {
public:

  /**
   * Default Constructor
   */
  ARLRobot();

  /**
   * Destructor
   */
  ~ARLRobot();

  /**
   * After creating instance of robot's hardware the communication device to the hardware needs to be initialized
   * @param nh NodeHandle of driver's node
   */
  void initialize(ros::NodeHandle nh);

  /**
   * Reads all muscle descriptions from parameter server and saves them into internal datastructure of robot interface
   * @param nh NodeHandle of driver's node
   */
  void getMuscleDescriptionsFromParameterServer(ros::NodeHandle nh);

  /**
   * If communication device needs cleanup before shutting down the driver this method should be called
   */
  void close();

  /**
   * Read current robot state from hardware
   * @param time current ROS time
   * @param period period of last control loop
   */
  void read(const ros::Time &time, const ros::Duration &period);

  /**
   * Write current commands to robot's hardware
   * @param time current ROS time
   * @param period period of last control loop
   */
  void write(const ros::Time &time, const ros::Duration &period);

private:
  arl_interfaces::MuscleInterface muscle_interface; /**< MuscleInterface for usage of MuscleController */
  bool initialized; /**< Saves if robot is already intialized*/
  CommunicationDevice *dev; /**< Handle for communication interface to robot's hardware */
  std::vector<std::string> names_; /**< Internal datastructure which contains all muscle's names */
  std::vector<double> desired_pressures_; /**< Internal datastructure which contains all muscle's desired pressures */
  std::vector<double> current_pressures_; /**< Internal datastructure which contains all muscle's current pressures */
  std::vector<double> tensions_; /**< Internal datastructure which contains all muscle's tensions */
  std::vector<double> activations_; /**< Internal datastructure which contains all normalized muscle's activation values */
  std::vector<std::map<std::string,std::pair<int,int> > > controller_ports;

  std::vector<double> last_activations_;

};

#endif // ROBOT_H
