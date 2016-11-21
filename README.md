# Driver package for the ARL Upper Limb Robot

## Index
1. Introduction
2. Components
	1. ARL Driver
	2. ARL RobotHW
	3. Communication Interface
	4. Utilized Interface and Handle
3. Usage
	1. Config Files
	2. Launch Files
4. Build Status

## Introduction
This package contains the RobotHW Interface and the device driver components for the ARL Upper Limb Robot.

## Components
The following paragraphs will introduce the individual components of this package.

### ARL Driver
The driver component contains the initialization of the controller manager which will load the controllers found in the [arl_controllers](https://github.com/arne48/arl_controllers) package.
Furthermore will it handle the initialization of the control-loop's realtime parameters and the ROS handles.

### ARL RobotHW
A custom Hardware Interface is implemented which contains the datastructures  needed for driving a Robot using PEMs.

### Communication Interface
The RobotHardware uses the implantations of this interface to communicate with the robot's hardware. This is abstracted into an own interface 
to keep the implementation of the communication protocols out of the RobotHW interface.


### Utilized Interface and Handle 
*MuscleHandle* from [arl_interfaces](https://github.com/arne48/arl_interfaces)

*MuscleInterface* from [arl_interfaces](https://github.com/arne48/arl_interfaces)

## Usage
The next lines will explain the content and usage of this package's  config and launch files.

### Config Files
* driver.yaml
    * using_raspberry_pi = bool <false> 
    This parameter switches the usage of the Raspberry Pi implementation to communicate with the hardware.
    * muscle_list\[\{name = string, initial_value = double \},..\]
    This describes all muscles controlled by this driver and also allows to set an initial pressure value.

### Launch Files
* arl_robot_driver.launch
First this file loads the muscle descriptions for this driver to the parameter server and then launches the driver node.

## Build Status
TBA