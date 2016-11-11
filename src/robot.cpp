#include <arl_hw/robot.h>

#include <wiringPi.h>

ARLRobot::ARLRobot() {
  initialized = false;
  initialize();
}

ARLRobot::~ARLRobot() {
  close();
}

void ARLRobot::initialize() {
  if (wiringPiSetup () == -1) {
    ROS_WARN("wasn't able to setup RPi - falling back to dummy interface");
  }else {

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
  ROS_DEBUG("READ with %f hz", 1 / period.toSec());
}

void ARLRobot::write(const ros::Time& time, const ros::Duration& period) {
  ROS_DEBUG("WRITE with %f hz", 1 / period.toSec());
}