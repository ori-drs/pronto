#pragma once
#include <Eigen/Dense>
#include <pronto_estimator_core/rbis_update_interface.hpp>
#include <bot_param/param_client.h>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <pronto_estimator_core/mav_state_est.hpp>
#include <pronto_estimator_core/scan_matcher_module.hpp>

namespace  MavStateEst {

class ScanMatcherHandler : SensingModule<bot_core::pose_t>{
public:
  ScanMatcherHandler(BotParam * param);

  RBISUpdateInterface * processMessage(const bot_core::pose_t * msg,
                                       MavStateEstimator* state_estimator) override;

  bool processMessageInit(const bot_core::pose_t *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov) override;

protected:
  ScanMatcherModule scan_matcher_module_;
  PoseMeasurement pose_meas_;

};


} // namespace MavStateEst
