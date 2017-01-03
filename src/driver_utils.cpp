#include <arl_hw/driver_utils.h>

namespace driver_utils {

  void publishDiagnostics(realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher, statistics_t driver_stats) {
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

      if (driver_stats.emergency_stop_engaged) {
        status.mergeSummaryf(status.WARN, "Emergency Stop Engaged");
      }

      statuses.push_back(status);
      publisher.msg_.status = statuses;
      publisher.msg_.header.stamp = ros::Time::now();
      publisher.unlockAndPublish();
    }
  }

  double get_now() {
    struct timespec n;
    clock_gettime(CLOCK_MONOTONIC, &n);
    return double(n.tv_nsec) / NSEC_PER_SECOND + n.tv_sec;
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

  driver_utils::statistics_t &
  checkOverrun(driver_utils::statistics_t &driver_stats, double start, double after_read, double after_cm, double after_write,
               int period_int) {
    struct timespec before;
    struct timespec ts = {0, 0};
    struct timespec tick;

    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND) > (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND)) {
      // Total amount of time the loop took to run
      driver_stats.overrun_loop_sec = (before.tv_sec + double(before.tv_nsec) / NSEC_PER_SECOND) -
                                      (tick.tv_sec + double(tick.tv_nsec) / NSEC_PER_SECOND);

      // We overran, snap to next "period"
      tick.tv_sec = before.tv_sec;
      tick.tv_nsec = (before.tv_nsec / period_int) * period_int;
      driver_utils::timespecInc(&tick, period_int);

      // initialize overruns
      if (driver_stats.overruns == 0) {
        driver_stats.last_overrun = 1000;
        driver_stats.last_severe_overrun = 1000;
      }

      // check for overruns
      if (driver_stats.recent_overruns > 10) {
        driver_stats.last_severe_overrun = 0;
      }
      driver_stats.last_overrun = 0;

      driver_stats.overruns++;
      driver_stats.recent_overruns++;
      driver_stats.overrun_read = after_read - start;
      driver_stats.overrun_cm = after_cm - after_read;
      driver_stats.overrun_write = after_write - after_cm;
    }
    return driver_stats;
  }

}
