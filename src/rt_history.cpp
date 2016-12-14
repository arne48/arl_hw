#include <arl_hw/rt_history.h>

RTLoopHistory::RTLoopHistory(unsigned length, double default_value) : index_(0), length_(length), history_(new double[length]) {
  for (unsigned i = 0; i < length_; ++i)
    history_[i] = default_value;
}

RTLoopHistory::~RTLoopHistory() {
  delete[] history_;
}

void RTLoopHistory::sample(double value) {
  index_ = (index_ + 1) % length_;
  history_[index_] = value;
}

double RTLoopHistory::average() const {
  double sum(0.0);
  for (unsigned i = 0; i < length_; ++i)
    sum += history_[i];
  return sum / double(length_);
}
