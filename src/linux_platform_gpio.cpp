#include <arl_hw/linux_platform_gpio.h>
#include <ros/ros.h>

LinuxPlatform_GPIO::LinuxPlatform_GPIO() {
}

LinuxPlatform_GPIO::~LinuxPlatform_GPIO() {}

void LinuxPlatform_GPIO::init(int gpio_address) {
  int fd, len;
  char buf[MAX_BUF];

  fd = open(GPIO_DIR "/export", O_WRONLY);
  if (fd < 0) {
    ROS_ERROR("Couldn't initialize GPIO port");
  }

  len = snprintf(buf, sizeof(buf), "%d", gpio_address);
  write(fd, buf, len);
  close(fd);
}

void LinuxPlatform_GPIO::deinit(int gpio_address) {
  int fd, len;
  char buf[MAX_BUF];

  fd = open(GPIO_DIR "/unexport", O_WRONLY);
  if (fd < 0) {
    ROS_ERROR("Couldn't de-initialize GPIO port");
  }

  len = snprintf(buf, sizeof(buf), "%d", gpio_address);
  write(fd, buf, len);
  close(fd);
}

void LinuxPlatform_GPIO::set_mode(int gpio_address, Embedded_GPIO::gpio_mode mode) {
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), GPIO_DIR  "/gpio%d/direction", gpio_address);

  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    ROS_ERROR("Couldn't set mode of GPIO port");
  }

  if (mode == Embedded_GPIO::gpio_mode::OUTPUT) {
    write(fd, "out", 4);
  } else {
    write(fd, "in", 3);
  }

  close(fd);
}

void LinuxPlatform_GPIO::set_output(int gpio_address, Embedded_GPIO::gpio_state state) {
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), GPIO_DIR "/gpio%d/value", gpio_address);

  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    ROS_ERROR("Couldn't set state of GPIO port");
  }

  if (state == Embedded_GPIO::gpio_state::ON) {
    write(fd, "1", 2);
  } else {
    write(fd, "0", 2);
  }

  close(fd);
}

Embedded_GPIO::gpio_state LinuxPlatform_GPIO::read_input(int gpio_address){
  int fd;
  char buf[MAX_BUF];
  char ch;
  Embedded_GPIO::gpio_state ret = Embedded_GPIO::gpio_state::OFF;

  snprintf(buf, sizeof(buf), GPIO_DIR "/gpio%d/value", gpio_address);

  fd = open(buf, O_RDONLY);
  if (fd < 0) {
    ROS_ERROR("Couldn't get state of GPIO port");
  }

  read(fd, &ch, 1);

  if (ch != '0') {
    ret = Embedded_GPIO::gpio_state::ON;
  } else {
    ret = Embedded_GPIO::gpio_state::OFF;
  }

  close(fd);
  return ret;
}