#pragma once
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <string>
#include <pronto_estimator_core/rbis_update_interface.hpp>
#include <lcmtypes/bot_core/ins_t.hpp>
#include <pronto_estimator_core/mav_state_est.hpp>
#include <pronto_estimator_core/sensing_module.hpp>
#include <pronto_estimator_core/ins_module.hpp>

namespace MavStateEst {

class InsHandler : public SensingModule<bot_core::ins_t>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  void create(BotParam * _param, BotFrames * _frames);
public:
    InsHandler();
    ~InsHandler();
  InsHandler(BotParam * _param, BotFrames * _frames);


  RBISUpdateInterface * processMessage(const bot_core::ins_t * msg,
                                       MavStateEstimator* state_estimator);

  bool processMessageInit(const bot_core::ins_t * msg,
                          const std::map<std::string, bool> & sensors_initialized,
                          const RBIS & default_state,
                          const RBIM & default_cov,
                          RBIS & init_state,
                          RBIM & init_cov);
  inline double getTimeStep() {
      return ins_module_.getTimeStep();
  }

protected:
  // channel is used to determine which signal to subscribe to:
  std::string channel;
  InsModule ins_module_;
  ImuMeasurement imu_meas_;

};

} // namespace MavStateEst
