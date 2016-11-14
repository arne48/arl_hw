#include <stdio.h>
#include <getopt.h>
#include <execinfo.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

#include <ros/ros.h>
#include <arl_hw/robot.h>
#include <controller_manager/controller_manager.h>


#define CLOCK_PRIO 0
#define CONTROL_PRIO 0

static pthread_t controlThread;
static pthread_attr_t controlThreadAttr;

struct timespec ts = {0, 0};
struct timespec tick;


void terminationHandler(int signal) {
  ROS_INFO("Received Signal %d", signal);
}

void timespecInc(struct timespec *tick, int ns) {
  tick->tv_nsec += ns;
  while (tick->tv_nsec >= 1e9) {
    tick->tv_nsec -= 1e9;
    tick->tv_sec++;
  }
}

void waitForNextControlLoop(struct timespec tick, int sampling_ns) {

  timespecInc(&tick, sampling_ns);

  struct timespec before;
  clock_gettime(CLOCK_REALTIME, &before);

  tick.tv_sec = before.tv_sec;
  tick.tv_nsec = (before.tv_nsec / sampling_ns) * sampling_ns;
  timespecInc(&tick, sampling_ns);

  clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
}

void *controlLoop(void *) {

  ros::NodeHandle nh;

  ARLRobot robot;
  robot.initialize(nh);

  controller_manager::ControllerManager cm(&robot, nh);

  // Set real-time scheduler
  struct sched_param thread_param;
  int policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  pthread_setschedparam(pthread_self(), policy, &thread_param);


  //Set update rate with ROS datatype in HZ
  ros::Rate rate(1000); //HZ

  //Check if RTC is available
  if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
    ROS_FATAL("Failed to poll real-time clock!");
  }

  //Time for use in controllers
  ros::Time
    last(ts.tv_sec, ts.tv_nsec),
    now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  // Get the next full second to start control loop at
  tick.tv_sec = tick.tv_sec;
  tick.tv_nsec = (tick.tv_nsec / int(rate.expectedCycleTime().toNSec()) + 1) * int(rate.expectedCycleTime().toNSec());
  clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

  while (nh.ok()) {

    // Get the time / period
    if (!clock_gettime(CLOCK_REALTIME, &ts)) {
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } else {
      ROS_FATAL("Failed to poll real-time clock!");
      break;
    }

    ROS_DEBUG("Loops ran at %fhz off by %dus", 1 / period.toSec(), int(period.toNSec() - rate.expectedCycleTime().toNSec()) / 1000);

    //Main update
    robot.read(now, period);
    cm.update(now, period);
    robot.write(now, period);

    //Wait for the loop to start and maintain fixed rate
    waitForNextControlLoop(tick, int(rate.expectedCycleTime().toNSec()));

  }

}

int main(int argc, char **argv) {

  // Keep the kernel from swapping us out
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
    perror("mlockall");
    return -1;
  }

  //Initialize ROS communication
  ros::init(argc, argv, "arl_driver_node");


  // Catch attempts to quit
  signal(SIGTERM, terminationHandler);
  signal(SIGINT, terminationHandler);
  signal(SIGHUP, terminationHandler);

  //Start thread
  int rv;
  if ((rv = pthread_create(&controlThread, &controlThreadAttr, controlLoop, 0)) != 0) {
    ROS_FATAL("Unable to create control thread: rv = %d", rv);
    exit(EXIT_FAILURE);
  }

  ros::spin();
  pthread_join(controlThread, (void **) &rv);

}