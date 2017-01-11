#ifndef ARL_HW_RT_HISTORY_H
#define ARL_HW_RT_HISTORY_H

/**
 * Inherited class of the PR2's ethercat realtime controller to
 * calculate the arithmetic mean of loop rate measurements
 * collected inside of the controller's loop.
 */
class RTLoopHistory {
public:
  /**
   * Constructor
   * @param length amount of historical samples to use in calculation of arithmetic mean
   * @param default_value assumed loop rate (hz) for initializing the history
   */
  RTLoopHistory(unsigned length, double default_value);

  /**
   * Destructor
   */
  ~RTLoopHistory();

  /**
   * Adds a new sample to the history
   * @param value loop rate
   */
  void sample(double value);

  /**
   * Calculates arithmetic mean of historic loop rates
   * @return arithmetic mean of last [length] loop rates
   */
  double average() const;

protected:
  unsigned index_;
  unsigned length_;
  double *history_;
};

#endif //ARL_HW_RT_HISTORY_H
