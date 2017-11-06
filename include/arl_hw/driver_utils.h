#ifndef ARL_HW_DRIVER_UTILS_H
#define ARL_HW_DRIVER_UTILS_H

#include <pthread.h>

#include <ros/ros.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <arl_hw_msgs/MusculatureState.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include <arl_hw/rt_history.h>
#include <arl_hw/robot.h>

#define NSEC_PER_SECOND 1e+9
#define USEC_PER_SECOND 1e6

using namespace boost::accumulators;

namespace driver_utils {

  /**
   * Datastructure to store statistical information about the driver performance
   */
  struct statistics_t {
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
    double halt_rt_loop_frequency;
    double rt_loop_frequency;
    bool rt_loop_not_making_timing;
    bool emergency_stop_engaged;
  };

  /**
   * Returns an initialized instance of driver statistics
   * @return
   */
  struct statistics_t init_driver_statistics();

  /**
   * Publishes diagnostic message based on driver statistics
   * @param publisher
   * @param driver_stats
   */
  void publishDiagnostics(realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher, statistics_t &driver_stats);

  /**
   * Returns combined value of us clock time and ns clock time
   * @return
   */
  double get_now();

  /**
   * Increases a time structure by a certain amount of nanoseconds
   * @param tick
   * @param ns
   */
  void timespecInc(struct timespec *timestamp, int ns);

  /**
   * Suspends the the calling thread to lock on loop frequency goal
   * @param tick
   * @param sampling_ns
   */
  void waitForNextControlLoop(struct timespec timestamp, int sampling_ns);

  /**
   * Checks if a loop took more time than it is due to it
   * @param driver_stats
   * @param start
   * @param after_read
   * @param after_cm
   * @param after_write
   * @param period_int
   * @param time
   */
  void checkOverrun(statistics_t &driver_stats, double start, double after_read, double after_cm, double after_write,
                    int period_int, struct timespec &timestamp);

  /**
   * Checks if a frequency miss was so severe that that driver needs to be halted.
   * In case of severe miss stops sets flag to halt driver from execution.
   * @param last_rt_monitor_time
   * @param rt_cycle_count
   * @param rt_loop_monitor_period
   * @param rt_loop_history
   * @param driver_stats
   * @param start
   * @param robot
   */
  void checkSevereRTMiss(double *last_rt_monitor_time, unsigned int *rt_cycle_count, double rt_loop_monitor_period,
                         RTLoopHistory &rt_loop_history,
                         driver_utils::statistics_t &driver_stats, double start, ARLRobot &robot);
}

#endif //ARL_HW_DRIVER_UTILS_H
