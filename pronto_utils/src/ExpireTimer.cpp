#include "pronto_utils/ExpireTimer.hpp"
#include <iostream>

namespace pronto {

ExpireTimer::ExpireTimer() {
  desired_uper = 0;
  uperiod = 0;
  firstpass = true;
  previous_uts = 0;
}

void ExpireTimer::setDesiredPeriod_us(const unsigned long &uper) {
  desired_uper = uper;
  reset();
}


void ExpireTimer::reset() {
  uperiod = desired_uper;
  firstpass = true;
}

bool ExpireTimer::processSample(const unsigned long long &uts) {
  if (firstpass) {
    // we need to grab the first timestamp here
    firstpass = false;
    previous_uts = uts;
  }

  if (uts < previous_uts) {
    std::cout << "ExpireTimer::processSample -- Jumping back in time, behavior unpredictable.\n";
    previous_uts = uts;
  }

  unsigned long delta;
  delta = (unsigned long)(uts - previous_uts);


  if (delta > uperiod) {
    // this would make the counter negative, so just saturate low
    delta = uperiod;
  }

  uperiod = uperiod - delta;
  previous_uts = uts;

  return getState();
}

unsigned long ExpireTimer::getRemainingTime_us() {
  return uperiod;
}

bool ExpireTimer::getState() {
  if (uperiod==0)
  {
    // negative values are not handled by this class. We check zero only
    return true;
  }
  return false;
}

}
