#include <stdio.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <pthread.h>

#include <ros/ros.h>
#include <arl_hw/robot.h>
#include <arl_hw/rt_history.h>
#include <controller_manager/controller_manager.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>


#define CLOCK_PRIO 0
#define CONTROL_PRIO 0
#define NSEC_PER_SECOND 1e+9
#define USEC_PER_SECOND 1e6


using namespace boost::accumulators;

static pthread_t controlThread;
static pthread_attr_t controlThreadAttr;

struct timespec ts = {0, 0};
struct timespec tick;

static struct {
  accumulator_set<double, stats<tag::max, tag::mean> > read_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > write_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > cm_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > loop_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > jitter_acc;
  int overruns;
  int recent_overruns;
  int last_overrun;
  int last_severe_overrun;
  double overrun_loop_sec;
  double overrun_read;
  double overrun_write;
  double overrun_cm;

  bool rt_loop_not_making_timing;
  double halt_rt_loop_frequency;
  double rt_loop_frequency;
} driver_stats;

static void publishDiagnostics(realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher) {
  if (publisher.trylock()) {
    accumulator_set<double, stats<tag::max, tag::mean> > zero;
    std::vector<diagnostic_msgs::DiagnosticStatus> statuses;
    diagnostic_updater::DiagnosticStatusWrapper status;

    static double max_read = 0, max_write = 0, max_cm = 0, max_loop = 0, max_jitter = 0;
    double avg_read, avg_write, avg_cm, avg_loop, avg_jitter;

    avg_read = extract_result<tag::mean>(driver_stats.read_acc);
    avg_write = extract_result<tag::mean>(driver_stats.write_acc);
    avg_cm = extract_result<tag::mean>(driver_stats.cm_acc);
    avg_loop = extract_result<tag::mean>(driver_stats.loop_acc);
    max_read = std::max(max_read, extract_result<tag::max>(driver_stats.read_acc));
    max_write = std::max(max_write, extract_result<tag::max>(driver_stats.write_acc));
    max_cm = std::max(max_cm, extract_result<tag::max>(driver_stats.cm_acc));
    max_loop = std::max(max_loop, extract_result<tag::max>(driver_stats.loop_acc));
    driver_stats.read_acc = zero;
    driver_stats.write_acc = zero;
    driver_stats.cm_acc = zero;
    driver_stats.loop_acc = zero;

    // Publish average loop jitter
    avg_jitter = extract_result<tag::mean>(driver_stats.jitter_acc);
    max_jitter = std::max(max_jitter, extract_result<tag::max>(driver_stats.jitter_acc));
    driver_stats.jitter_acc = zero;

    status.addf("Max writing roundtrip of controllers (us)", "%.2f", max_write * USEC_PER_SECOND);
    status.addf("Avg writing roundtrip of controllers (us)", "%.2f", avg_write * USEC_PER_SECOND);
    status.addf("Max reading roundtrip of controllers (us)", "%.2f", max_read * USEC_PER_SECOND);
    status.addf("Avg reading roundtrip of controllers (us)", "%.2f", avg_read * USEC_PER_SECOND);
    status.addf("Max Controller Manager roundtrip (us)", "%.2f", max_cm * USEC_PER_SECOND);
    status.addf("Avg Controller Manager roundtrip (us)", "%.2f", avg_cm * USEC_PER_SECOND);
    status.addf("Max Total Loop roundtrip (us)", "%.2f", max_loop * USEC_PER_SECOND);
    status.addf("Avg Total Loop roundtrip (us)", "%.2f", avg_loop * USEC_PER_SECOND);
    status.addf("Max Loop Jitter (us)", "%.2f", max_jitter * USEC_PER_SECOND);
    status.addf("Avg Loop Jitter (us)", "%.2f", avg_jitter * USEC_PER_SECOND);
    status.addf("Control Loop Overruns", "%d", driver_stats.overruns);
    status.addf("Recent Control Loop Overruns", "%d", driver_stats.recent_overruns);
    status.addf("Last Control Loop Overrun Cause", "read: %.2fus, cm: %.2fus, write: %.2fus",
                driver_stats.overrun_read * USEC_PER_SECOND, driver_stats.overrun_cm * USEC_PER_SECOND,
                driver_stats.overrun_write * USEC_PER_SECOND);
    status.addf("Last Overrun Loop Time (us)", "%.2f", driver_stats.overrun_loop_sec * USEC_PER_SECOND);
    status.addf("Realtime Loop Frequency", "%.4f", driver_stats.rt_loop_frequency);

    status.name = "Realtime Control Loop";
    if (driver_stats.overruns > 0 && driver_stats.last_overrun < 30) {
      if (driver_stats.last_severe_overrun < 30)
        status.level = 1;
      else
        status.level = 0;
      status.message = "Realtime loop used too much time in the last 30 seconds.";
    } else {
      status.level = 0;
      status.message = "OK";
    }
    driver_stats.recent_overruns = 0;
    driver_stats.last_overrun++;
    driver_stats.last_severe_overrun++;

    if (driver_stats.rt_loop_not_making_timing) {
      status.mergeSummaryf(status.ERROR, "Halting, realtime loop only ran at %.4f Hz", driver_stats.halt_rt_loop_frequency);
    }

    statuses.push_back(status);
    publisher.msg_.status = statuses;
    publisher.msg_.header.stamp = ros::Time::now();
    publisher.unlockAndPublish();
  }
}


static inline double get_now() {
  struct timespec n;
  clock_gettime(CLOCK_MONOTONIC, &n);
  return double(n.tv_nsec) / NSEC_PER_SECOND + n.tv_sec;
}

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

  double last_published, last_loop_start, last_rt_monitor_time;
  unsigned rt_cycle_count = 0;
  double rt_loop_monitor_period = 0.6 / 3;
  RTLoopHistory rt_loop_history(3, 1000.0);

  ros::NodeHandle nh;

  ARLRobot robot;
  robot.initialize(nh);

  // Realtime loop should be running at least 1000Hz
  double min_acceptable_rt_loop_frequency = 1000.0;
  nh.param<double>("/min_acceptable_rt_loop_frequency", min_acceptable_rt_loop_frequency, 1000.0);

  bool publish_rt_jitter = false;
  nh.param<bool>("/publish_every_rt_jitter", publish_rt_jitter, false);

  controller_manager::ControllerManager cm(&robot, nh);

  realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> publisher(nh, "/diagnostics", 2);
  realtime_tools::RealtimePublisher<std_msgs::Float64> *rtpublisher =
    new realtime_tools::RealtimePublisher<std_msgs::Float64>(nh, "/realtime_jitter", 2);


  // Publish one-time before entering real-time to pre-allocate message vectors
  publishDiagnostics(publisher);

  // Set real-time scheduler
  struct sched_param thread_param;
  int policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  pthread_setschedparam(pthread_self(), policy, &thread_param);


  //Set update rate with ROS datatype in HZ
  ros::Rate rate(1000); //Hz

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


  last_published = get_now();
  last_rt_monitor_time = get_now();
  last_loop_start = get_now();
  while (nh.ok()) {

    // Track how long the actual loop takes
    double this_loop_start = get_now();
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
    double start = get_now();
    robot.read(now, period);
    double after_read = get_now();
    cm.update(now, period);
    double after_cm = get_now();
    robot.write(now, period);
    double after_write = get_now();

    //Accumulate section's durations of update cycle
    driver_stats.cm_acc(after_cm - after_read);
    driver_stats.write_acc(after_write - start);
    driver_stats.read_acc(after_write - after_cm);

    double end = get_now();
    if ((end - last_published) > 1.0) {
      publishDiagnostics(publisher);
      last_published = end;
    }


    // Realtime loop should run about 1000Hz.
    // Missing timing on a control cycles usually causes a controller glitch and actuators to jerk.
    // When realtime loop misses a lot of cycles controllers will perform poorly and may cause robot to shake.
    ++rt_cycle_count;
    if ((start - last_rt_monitor_time) > rt_loop_monitor_period) {
      // Calculate new average rt loop frequency
      double rt_loop_frequency = double(rt_cycle_count) / rt_loop_monitor_period;

      // Use last X samples of frequency when deciding whether or not to halt
      rt_loop_history.sample(rt_loop_frequency);
      double avg_rt_loop_frequency = rt_loop_history.average();
      if (avg_rt_loop_frequency < min_acceptable_rt_loop_frequency) {
        driver_stats.halt_rt_loop_frequency = avg_rt_loop_frequency;
        driver_stats.rt_loop_not_making_timing = true;
      }
      driver_stats.rt_loop_frequency = avg_rt_loop_frequency;
      rt_cycle_count = 0;
      last_rt_monitor_time = start;
    }

    // Compute end of next period
    int period_int = int(rate.expectedCycleTime().toNSec());
    timespecInc(&tick, period_int);

    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND) > (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND)) {
      // Total amount of time the loop took to run
      driver_stats.overrun_loop_sec = (before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND) -
                                      (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND);

      // We overran, snap to next "period"
      tick.tv_sec = before.tv_sec;
      tick.tv_nsec = (before.tv_nsec / period_int) * period_int;
      timespecInc(&tick, period_int);

      // initialize overruns
      if (driver_stats.overruns == 0) {
        driver_stats.last_overrun = 1000;
        driver_stats.last_severe_overrun = 1000;
      }
      // check for overruns
      if (driver_stats.recent_overruns > 10)
        driver_stats.last_severe_overrun = 0;
      driver_stats.last_overrun = 0;

      driver_stats.overruns++;
      driver_stats.recent_overruns++;
      driver_stats.overrun_read = after_read - start;
      driver_stats.overrun_cm = after_cm - after_read;
      driver_stats.overrun_write = after_write - after_cm;
    }

    //Wait for the loop to start and maintain fixed rate
    waitForNextControlLoop(tick, int(rate.expectedCycleTime().toNSec()));

    // Calculate RT loop jitter
    struct timespec after;
    clock_gettime(CLOCK_REALTIME, &after);
    double jitter = (after.tv_sec - tick.tv_sec + double(after.tv_nsec - tick.tv_nsec) / NSEC_PER_SECOND);

    driver_stats.jitter_acc(jitter);

    // Publish realtime loops statistics
    if (publish_rt_jitter && rtpublisher) {
      if (rtpublisher->trylock()) {
        rtpublisher->msg_.data = jitter;
        rtpublisher->unlockAndPublish();
      }
    }
  }

  delete rtpublisher;
  robot.close();

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