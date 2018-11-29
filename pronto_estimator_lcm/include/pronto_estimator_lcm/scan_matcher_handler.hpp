#pragma once
#include <Eigen/Dense>
#include <pronto_estimator_core/rbis_update_interface.hpp>
#include <bot_param/param_client.h>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <pronto_estimator_core/mav_state_est.hpp>
#include <pronto_estimator_core/scan_matcher_module.hpp>

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
