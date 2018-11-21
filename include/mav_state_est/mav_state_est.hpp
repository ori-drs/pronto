#ifndef MAV_STATE_ESTIMATOR_HPP_
#define MAV_STATE_ESTIMATOR_HPP_

#include "rbis.hpp"
#include "update_history.hpp"

namespace MavStateEst {

class MavStateEstimator {
public:
  MavStateEstimator(RBISResetUpdate * init_state,
                    const uint64_t& history_span);
  ~MavStateEstimator();

  updateHistory::historyMapIterator unprocessed_updates_start;
  updateHistory history;

  uint64_t utime_history_span;

  void addUpdate(RBISUpdateInterface * update, bool roll_forward);
  void getHeadState(RBIS & head_state, RBIM & head_cov);
  double getMeasurementsLogLikelihood();
  void EKFSmoothBackwardsPass(double dt);

};

}

#endif /* MAV_STATE_ESTIMATOR_HPP_ */
