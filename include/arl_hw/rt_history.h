#ifndef ARL_HW_RT_HISTORY_H
#define ARL_HW_RT_HISTORY_H


class RTLoopHistory {
public:
  RTLoopHistory(unsigned length, double default_value);

  ~RTLoopHistory();

  void sample(double value);

  double average() const;

protected:
  unsigned index_;
  unsigned length_;
  double *history_;
};

#endif //ARL_HW_RT_HISTORY_H
