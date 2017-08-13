#include <cstdio>
#include <csignal>
#include <sys/mman.h>
#include <sys/types.h>
#include <pthread.h>

#include <ros/ros.h>
#include <arl_hw/robot.h>
#include <arl_hw/rt_history.h>
#include <arl_hw/driver_utils.h>
#include <controller_manager/controller_manager.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#define RATE 500 //Hz

static pthread_t controlThread;
static pthread_attr_t controlThreadAttr;
ARLRobot robot;

void *controlLoop(void *) {
  struct timespec ts = {0, 0};
  struct timespec tick;

  double last_published, last_loop_start, last_rt_monitor_time;
  unsigned rt_cycle_count = 0;
  double rt_loop_monitor_period = 0.6 / 3;
  RTLoopHistory rt_loop_history(3, RATE);
  driver_utils::statistics_t driver_stats;

  ros::NodeHandle nh;

  robot.initialize(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> publisher(nh, "/diagnostics", 2);
  realtime_tools::RealtimePublisher<std_msgs::Float64> *rtpublisher =
    new realtime_tools::RealtimePublisher<std_msgs::Float64>(nh, "/realtime_jitter", 2);


  // Publish one-time before entering real-time to pre-allocate message vectors
  driver_utils::publishDiagnostics(publisher, driver_stats);

  // Set real-time scheduler
  struct sched_param thread_param;
  int policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  pthread_setschedparam(pthread_self(), policy, &thread_param);


  //Set update rate with ROS datatype in HZ
  ros::Rate rate(RATE);

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


  last_published = driver_utils::get_now();
  last_rt_monitor_time = driver_utils::get_now();
  last_loop_start = driver_utils::get_now();

  while (nh.ok()) {

    // Check for Emergency Halt
    driver_stats.emergency_stop_engaged = robot.emergency_stop;

    double start;
    double after_read;
    double after_cm;
    double after_write;

    if (driver_stats.emergency_stop_engaged) {
      start = driver_utils::get_now();
      after_read = start;
      after_cm = start;
      after_write = start;

      robot.executeEmergencyStop();
      cm.update(now, period);

      double end = driver_utils::get_now();
      if ((end - last_published) > 1.0) {
        driver_utils::publishDiagnostics(publisher, driver_stats);
        last_published = end;
      }

    } else {
      // Track how long the actual loop takes
      double this_loop_start = driver_utils::get_now();
      driver_stats.loop_acc(this_loop_start - last_loop_start);
      last_loop_start = this_loop_start;

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

      //Main update
      start = driver_utils::get_now();

      robot.read(now, period);
      after_read = driver_utils::get_now();

      cm.update(now, period);
      after_cm = driver_utils::get_now();

      //Just write new data to muscle controllers as long as the rt loop is stable
      if (!driver_stats.rt_loop_not_making_timing) {
        robot.write(now, period);
      }
      after_write = driver_utils::get_now();

      //Accumulate section's durations of update cycle
      driver_stats.read_acc(after_read - start);
      driver_stats.cm_acc(after_cm - after_read);
      driver_stats.write_acc(after_write - after_cm);

      double end = driver_utils::get_now();
      if ((end - last_published) > 1.0) {
        driver_utils::publishDiagnostics(publisher, driver_stats);
        last_published = end;
      }

      driver_utils::checkSevereRTMiss(&last_rt_monitor_time, &rt_cycle_count, rt_loop_monitor_period, rt_loop_history, driver_stats, start,
                                      robot);
    }

    // Compute end of next period
    int period_int = int(rate.expectedCycleTime().toNSec());
    driver_utils::timespecInc(&tick, period_int);

    driver_utils::checkOverrun(driver_stats, start, after_read, after_cm, after_write, period_int, tick);

    //Wait for the loop to start and maintain fixed rate
    driver_utils::waitForNextControlLoop(tick, int(rate.expectedCycleTime().toNSec()));

    // Calculate RT loop jitter
    struct timespec after;
    clock_gettime(CLOCK_REALTIME, &after);
    double jitter = (after.tv_sec - tick.tv_sec + double(after.tv_nsec - tick.tv_nsec) / NSEC_PER_SECOND);
    driver_stats.jitter_acc(jitter);

    // Publish realtime loops current jitter
    if (robot.driver_config.publish_every_rt_jitter && rtpublisher->trylock()) {
      rtpublisher->msg_.data = jitter;
      rtpublisher->unlockAndPublish();
    }
  }

  delete rtpublisher;
  robot.close();

}

bool resetMusclesService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
  robot.resetMuscles();
  return true;
}

bool emergencyStopService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp) {
  robot.emergency_stop = req.data;
  return true;
}

int main(int argc, char **argv) {

  // Keep the kernel from swapping us out
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
    perror("mlockall");
    return -1;
  }

  //Initialize ROS communication
  ros::init(argc, argv, "arl_driver_node");
  ros::NodeHandle nh;

  ros::ServiceServer reset = nh.advertiseService("/reset_muscles", resetMusclesService);
  ros::ServiceServer halt = nh.advertiseService("/emergency_stop", emergencyStopService);

  //Start thread
  int rv;
  if ((rv = pthread_create(&controlThread, &controlThreadAttr, controlLoop, 0)) != 0) {
    ROS_FATAL("Unable to create control thread: rv = %d", rv);
    exit(EXIT_FAILURE);
  }

  ros::spin();
  pthread_join(controlThread, (void **) &rv);

}