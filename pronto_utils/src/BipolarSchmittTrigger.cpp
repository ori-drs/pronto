#include "pronto_utils/BipolarSchmittTrigger.hpp"
#include <iostream>
#include <cmath>

namespace pronto {

void BipolarSchmittTrigger::UpdateState(long present_time, double value) {
  // Think we need to reset on a sign flip event
  //if (sign(value) != sign(prev_value)) {
  //	_trigger->Reset();
  //}

  _trigger->updateState(present_time, std::abs(value));
  //_lowside->UpdateState(present_time, value);
  prev_value = value;
}

void BipolarSchmittTrigger::Reset() {
  _trigger->reset();
  //_lowside->Reset();
}

float BipolarSchmittTrigger::getState() {
  if (_trigger->getState() > 0.9f)
    return 1.f;
  return 0.f;
}

BipolarSchmittTrigger::BipolarSchmittTrigger(double lt, double ht, long low_delay, long high_delay) {
  _trigger = new SchmittTrigger(lt, ht, low_delay, high_delay);
  //_lowside  = new SchmittTrigger(-lt, -ht, delay);

  prev_value = 0.;
}

BipolarSchmittTrigger::~BipolarSchmittTrigger() {
  std::cout << "Closing out a BipolarSchmittTrigger object\n";

  delete _trigger;
  //delete _lowside;
}

} // namespace pronto
