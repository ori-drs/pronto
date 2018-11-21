#pragma once
#include <Eigen/Dense>
#include "mav_state_est/rbis_update_interface.hpp"
#include <bot_param/param_client.h>
#include <lcmtypes/bot_core/pose_t.hpp>
#include "mav_state_est/mav_state_est.hpp"

namespace  MavStateEst {

class ScanMatcherHandler {
public:
  typedef enum {
    MODE_POSITION, MODE_POSITION_YAW, MODE_VELOCITY, MODE_VELOCITY_YAW, MODE_YAW
  } ScanMatchingMode;

  ScanMatcherHandler(BotParam * param);
  RBISUpdateInterface * processMessage(const bot_core::pose_t * msg, MavStateEstimator* state_estimator);

  ScanMatchingMode mode;
  Eigen::VectorXi z_indices;
  Eigen::MatrixXd cov_scan_match;
};


} // namespace MavStateEst
