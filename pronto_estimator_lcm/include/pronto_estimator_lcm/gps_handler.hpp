#pragma once
#include <pronto_estimator_core/rbis_update_interface.hpp>
#include <pronto_estimator_core/mav_state_est.hpp>
#include <pronto_estimator_core/gps_module.hpp>
#include <lcmtypes/bot_core/gps_data_t.hpp>
#include <bot_param/param_client.h>
#include <memory>

namespace MavStateEst {
class GpsHandler : public SensingModule<bot_core::gps_data_t> {
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
protected:
  Eigen::Matrix3d cov_xyz;
  std::shared_ptr<GPSModule> gps_module_;
  GPSMeasurement gps_meas_;
};
} // namespace MavStateEst
