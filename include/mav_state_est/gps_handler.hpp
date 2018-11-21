#pragma once
#include "mav_state_est/rbis_update_interface.hpp"
#include "mav_state_est/mav_state_est.hpp"
#include <lcmtypes/bot_core/gps_data_t.hpp>
#include <bot_param/param_client.h>

namespace MavStateEst {
class GpsHandler {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  public:
  GpsHandler(BotParam * _param);
  RBISUpdateInterface * processMessage(const bot_core::gps_data_t * msg,
                                       MavStateEstimator* state_estimator);
  bool processMessageInit(const bot_core::gps_data_t * msg,
                          const std::map<std::string, bool> & sensors_initialized,
                          const RBIS & default_state,
                          const RBIM & default_cov,
                          RBIS & init_state,
                          RBIM & init_cov);
private:
  Eigen::Matrix3d cov_xyz;
};
} // namespace MavStateEst
