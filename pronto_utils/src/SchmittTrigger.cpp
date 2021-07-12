#include <pronto_utils/SchmittTrigger.hpp>
#include <iostream>

namespace pronto {

SchmittTrigger::SchmittTrigger() {
  reset();
}

SchmittTrigger::SchmittTrigger(double lt, double ht, long low_delay, long high_delay) {
  setParameters(lt, ht, low_delay, high_delay);
}

void SchmittTrigger::setParameters(double lt, double ht, long low_delay, long high_delay) {
  falling_edge_threshold = lt;
  rising_edge_threshold = ht;
  falling_edge_time_delay = low_delay;
  rising_edge_time_delay = high_delay;
  reset();
}

void SchmittTrigger::reset() {
  current_status = false;
  previous_time = 0; // how to handle this when it gets called for the first time.. should we have a flag and then a reset status
  timer = 0;
  stored_value = 0;

  // first call flag is used to intialize the timer, to enable delay measuring
  first_call = true;
}
void SchmittTrigger::forceHigh() {
  current_status = true;
  timer = 0;
}

void SchmittTrigger::forceLow() {
  current_status = false;
  timer = 0;
}

void SchmittTrigger::updateState(uint64_t present_time, double value) {
  if (first_call) {
    first_call = false;
    previous_time = present_time;
  }
  if (present_time < previous_time) {
    std::cerr << "SchmittTrigger::UpdateState -- Warning object is jumping back in time -- NOT updating.\n";
    return;
  }

  bool verbose = false;

  stored_value = value;

  if(verbose){
    std::cout << "ST: " << value << " N , timer: " << timer << " ns, status is: " << current_status << std::endl;
  }

  // The timing logic should be rewritten with ExpireTimer at the next opportune moment
  if (current_status) {
    if (value <= falling_edge_threshold){
      if(verbose) std::cout << "below threshold\n";
      if (timer > falling_edge_time_delay) {
        if (verbose) std::cout << "high state -> low trigger\n";
        current_status = false;
      } else {
        if (verbose) std::cout << "high state but clock rising for low\n";
        timer += (present_time - previous_time);
      }
    } else {
      if (verbose) std::cout << "high state, clock zero\n";
      timer = 0;
    }
  } else {
    if (value >= rising_edge_threshold) {
      if(verbose){
        std::cout << "above threshold\n";
        std::cout << "TIMER: " << timer << std::endl;
        std::cout << "NOW - previous" << (present_time - previous_time) << std::endl;
      }
      if (timer > rising_edge_time_delay) {
        if (verbose) std::cout << "low state -> high trigger\n";
        current_status = true;
      } else {
        if (verbose) std::cout << "low state but clock rising for high\n";
        timer += (present_time - previous_time);
      }
    } else {
      if (verbose) std::cout << "low state, clock zero\n";
      timer = 0;
    }
  }
  previous_time = present_time;
}

bool SchmittTrigger::getState() {
  return current_status;
}

double SchmittTrigger::getCurrentValue() {
  return stored_value;
}
}  // namespace pronto
