#pragma once
#include <cstdint>

namespace pronto {
class SchmittTrigger {
public:
  SchmittTrigger();
  SchmittTrigger(double lt, double ht, long low_delay, long high_delay);
  void setParameters(double lt, double ht, long low_delay, long high_delay);
  void updateState(uint64_t present_time, double value);
  void reset();
  bool getState();
  double getCurrentValue();
  void forceHigh();
  void forceLow();

private:
  bool current_status;
  double stored_value;
  uint64_t timer;
  uint64_t falling_edge_time_delay;
  uint64_t rising_edge_time_delay;
  uint64_t previous_time;
  double falling_edge_threshold;
  double rising_edge_threshold;
  bool first_call;
};
}  // namespace pronto
