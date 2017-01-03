#ifndef ARL_HW_DRIVER_UTILS_H
#define ARL_HW_DRIVER_UTILS_H

#include <pthread.h>

#include <ros/ros.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#define NSEC_PER_SECOND 1e+9
#define USEC_PER_SECOND 1e6

using namespace boost::accumulators;

namespace driver_utils {

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

  void publishDiagnostics(realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher, statistics_t &driver_stats);

  double get_now();

  void timespecInc(struct timespec *tick, int ns);

  void waitForNextControlLoop(struct timespec tick, int sampling_ns);

  void
  checkOverrun(statistics_t &driver_stats, double start, double after_read, double after_cm, double after_write,
               int period_int, struct timespec &tick);
}

#endif //ARL_HW_DRIVER_UTILS_H
