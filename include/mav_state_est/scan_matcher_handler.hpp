#pragma once
#include <Eigen/Dense>
#include "mav_state_est/rbis_update_interface.hpp"
#include <bot_param/param_client.h>
#include <lcmtypes/bot_core/pose_t.hpp>
#include "mav_state_est/mav_state_est.hpp"
#include "mav_state_est/scan_matcher_module.hpp"

namespace  MavStateEst {

class ScanMatcherHandler {
public:
  ScanMatcherHandler(BotParam * param);
  RBISUpdateInterface * processMessage(const bot_core::pose_t * msg, MavStateEstimator* state_estimator);

protected:
  ScanMatcherModule scan_matcher_module_;
  PoseMeasurement pose_meas_;

};


} // namespace MavStateEst
