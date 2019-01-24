#ifndef MAV_STATE_ESTIMATOR_HPP_
#define MAV_STATE_ESTIMATOR_HPP_

#include "rbis.hpp"
#include "update_history.hpp"
#include "pronto_core/filter_state_history.hpp"

namespace MavStateEst {

class MavStateEstimator {
public:
    typedef Eigen::Vector3d Vector3d;
    typedef Eigen::Quaterniond Quaterniond;
public:
  MavStateEstimator(RBISResetUpdate * init_state,
                    const uint64_t& history_span);
  ~MavStateEstimator();

  updateHistory::historyMapIterator unprocessed_updates_start;
  updateHistory history;

  uint64_t utime_history_span;

  void addUpdate(RBISUpdateInterface * update, bool roll_forward);
  void getHeadState(RBIS & head_state, RBIM & head_cov) const;

  void getHeadState(const uint64_t& utime,
                    RBIS &head_state,
                    RBIM &head_cov) const;

  bool getInterpolatedPose(const uint64_t& utime,
                           Vector3d&  position,
                           Quaterniond& orientation) const;

  bool getInterpolatedPose(const uint64_t &utime, Eigen::Isometry3d& pose) const;

  double getMeasurementsLogLikelihood() const;
  void EKFSmoothBackwardsPass(double dt);

};

}

#endif /* MAV_STATE_ESTIMATOR_HPP_ */
