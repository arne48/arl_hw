#ifndef ARL_HW_ROBOT_H
#define ARL_HW_ROBOT_H


#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/Bool.h>

#include <arl_hw/datatypes.h>
#include <arl_hw/communication_device.h>
#include <arl_hw_msgs/MusculatureCommand.h>

#ifdef FOR_RPI
#include <arl_hw/raspberry_pi.h>
#endif

#ifdef FOR_TINKERBOARD
#include <arl_hw/tinkerboard.h>
#endif

#ifdef FOR_LINUX_PLATFORM
#include <arl_hw/linux_platform.h>
#endif

#include <arl_hw/dummy.h>
#include <arl_interfaces/muscle_interface.h>
#include <arl_controllers/muscle_controller.h>

#include <set>
#include <vector>
#include <string>
#include <map>


/**
 * Implementation of Robot Hardware Interface for ARL's pneumatic muscle driven robots
 */
class ARLRobot : public hardware_interface::RobotHW {
public:

  struct config{
    std::string platform;
    bool halt_on_slow_rt_loop;
    bool publish_every_rt_jitter;
    double min_acceptable_rt_loop_frequency;
  };

  struct muscle_info_t{
    std::string name;
    double activation;
    double current_pressure;
    double desired_pressure;
    double tension;
    double tension_filtered;
    uint8_t control_mode;
  };

  struct analog_input_info_t {
    std::string name;
    double voltage;
  };

  config driver_config;
  bool emergency_stop;

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
   * Reads all configurations from parameter server and saves them into internal datastructures of robot interface
   * @param nh NodeHandle of driver's node
   */
  void getConfigurationFromParameterServer(ros::NodeHandle nh);

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

  /**
   * Blows of air of all registered muscles
   */
  void executeEmergencyStop();

  /**
   * Resets muscles to their default configurations and blows of air from them
   */
  void resetMuscles();

  /**
   * Resets a specific muscle
   * @param name
   */
  void resetMuscle(std::string name);

  struct muscle_info_t getMuscleInfo(unsigned long index);

  unsigned long getNumberOfMuscles();

  unsigned long getNumberOfAnalogInputs();

  struct analog_input_info_t getAnalogInputInfo(unsigned long index);

  void updateMuscleValues(arl_hw_msgs::MusculatureCommand musculature_command);

private:
  arl_interfaces::MuscleInterface muscle_interface; /**< MuscleInterface for usage of MuscleController */
  bool initialized; /**< Saves if robot is already intialized*/
  CommunicationDevice *dev_; /**< Handle for communication interface to robot's hardware */
  std::vector<std::string> names_; /**< Internal datastructure which contains all muscle's names */
  std::vector<std::string> analog_input_names_; /**< Internal datastructure which contains all generic analog input's names */
  std::vector<double> desired_pressures_; /**< Internal datastructure which contains all muscle's desired pressures */
  std::vector<double> current_pressures_; /**< Internal datastructure which contains all muscle's current pressures */
  std::vector<double> tensions_; /**< Internal datastructure which contains all muscle's tensions */
  std::vector<double> activations_; /**< Internal datastructure which contains all normalized muscle's activation values */
  std::vector<double> tensions_filtered_;
  std::vector<double> analog_inputs_;
  std::vector<uint8_t> control_modes_;
  std::map<std::string, unsigned int> index_map_;
  std::vector<std::pair<int, int> > activation_controllers_; /**< Internal datastructure which contains the address (Controller Port[Chip-Select Id] : Channel on Controller) for every muscle's activation controller*/
  std::vector<std::pair<int,int> > pressure_controllers_; /**< Internal datastructure which contains the address (Controller Port[Chip-Select Id] : Channel on Controller) for every muscle's pressure controller*/
  std::vector<std::pair<int,int> > tension_controllers_; /**< Internal datastructure which contains the address (Controller Port[Chip-Select Id] : Channel on Controller) for every muscle's tension controller*/
  std::vector<std::pair<int,int> > analog_input_controllers_; /**< Internal datastructure which contains the address (Controller Port[Chip-Select Id] : Channel on Controller) for every generic analog input*/
  std::vector<double> last_activations_; /**< Internal datastructure last activation values to prevent wasting time on setting the same value multiple times */
  std::set<int> pressure_ports; /**< Internal datastructure to store pressure controller ids as unique set */
  std::set<int> tension_ports; /**< Internal datastructure to store tension controller ids as unique set */
  std::vector<arl_datatypes::muscle_status_data_t> muscle_status_; /**< Internal datastructure to store the status of registered muscles */
  std::vector<arl_datatypes::analog_input_status_data_t> analog_input_status_;

};

#endif //ARL_HW_ROBOT_H
